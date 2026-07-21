# layout.py
# ---------
# Takes the output of parser.py's parse_structs()/parse_unions() and simulates the C
# compiler's layout rules to produce Python `struct` module format strings.

import re

# type_name -> (struct-module format char, size in bytes, alignment in bytes)
_TYPE_MAP = {
    "uint8_t": ("B", 1, 1),
    "int8_t": ("b", 1, 1),
    "uint16_t": ("H", 2, 2),
    "int16_t": ("h", 2, 2),
    "uint32_t": ("I", 4, 4),
    "int32_t": ("i", 4, 4),
    "uint64_t": ("Q", 8, 8),
    "int64_t": ("q", 8, 8),
    "float": ("f", 4, 4),
    "double": ("d", 8, 8),
    "char": ("c", 1, 1),
    "bool": ("?", 1, 1),
}

# Matches a field name with a trailing array size, e.g. "data[OTA_DATA_CHUNK]"
# group(1) = "data", group(2) = "OTA_DATA_CHUNK"
_ARRAY_RE = re.compile(r"(\w+)\[(\w+)\]$")


def _union_layout(
    name: str,
    unions: dict[str, list[tuple[str, str]]],
    struct_layouts: dict[str, tuple[str, int, int]],
    union_cache: dict[str, tuple[int, int]],
    defines: dict[str, int],
) -> tuple[int, int]:
    """Computes (size_bytes, alignment_bytes) for union `name`, the way C
    does: a union's size is its LARGEST member's size (rounded up to the
    union's own alignment), not a sum - every member overlaps at offset 0.

    Computed lazily and cached, so it can be called mid-way through
    build_formats() the first time a struct field references this union.
    At that point everything this union's members depend on is guaranteed
    to already be resolvable, since C requires a union's member types to
    be fully defined before the union itself is used anywhere.
    """
    if name in union_cache:
        return union_cache[name]

    if name not in unions:
        raise ValueError(f"union {name} was referenced but never parsed")

    max_size: int = 0
    max_align: int = 1
    for type_, var in unions[name]:
        m = _ARRAY_RE.match(var)
        var_name = m.group(1) if m else var
        array_len_raw = m.group(2) if m else None

        _, size, align = _resolve_type(
            type_, struct_layouts, unions, union_cache, defines
        )
        count = _resolve_count(array_len_raw, defines, f"union {name}.{var_name}")

        max_size = max(max_size, size * count)
        max_align = max(max_align, align)

    padded_size = max_size + (-max_size) % max_align
    union_cache[name] = (padded_size, max_align)
    return union_cache[name]


def _resolve_type(
    type_: str,
    struct_layouts: dict[str, tuple[str, int, int]],
    unions: dict[str, list[tuple[str, str]]],
    union_cache: dict[str, tuple[int, int]],
    defines: dict[str, int],
) -> tuple[str | None, int, int]:
    """Looks up (format_char, size, align) for one field's type_ string.

    type_ still has its "struct "/"union " prefix if it had one in the
    header (that's what disambiguates a nested struct/union field from a
    primitive of the same bare name).

    For a union field, format_char is None - a union's bytes don't have
    one fixed interpretation (which member is "active" depends on runtime
    state, e.g. a packet-type field elsewhere), so callers represent it as
    an opaque `{size}s` bytes blob in the format string instead.
    """
    if type_.startswith("struct "):
        nested = type_.split(" ", 1)[1]
        if nested not in struct_layouts:
            raise ValueError(
                f"struct {nested} not parsed yet - it must be defined "
                "earlier in the header than whatever references it"
            )
        fmt_char, size, align = struct_layouts[nested]
        return fmt_char, size, align

    if type_.startswith("union "):
        nested = type_.split(" ", 1)[1]
        size, align = _union_layout(
            nested, unions, struct_layouts, union_cache, defines
        )
        return None, size, align

    if type_ in _TYPE_MAP:
        return _TYPE_MAP[type_]

    raise ValueError(f"unknown type '{type_}' (not in _TYPE_MAP, not a parsed struct)")


def _resolve_count(
    array_len_raw: str | None, defines: dict[str, int], context: str
) -> int:
    """Resolves a field's array length to an int: 1 for a non-array field,
    the literal value for `field[8]`, or a lookup in `defines` for
    `field[SOME_MACRO]`. `context` is only used to name the field in the
    error message if a macro name can't be resolved.
    """
    if array_len_raw is None:
        return 1
    if array_len_raw.isdigit():
        return int(array_len_raw)
    if array_len_raw in defines:
        return defines[array_len_raw]
    raise ValueError(
        f"{context}: array size '{array_len_raw}' is a macro not present "
        "in `defines` - pass its resolved value in, e.g. via parse_defines()"
    )


def build_formats(
    structs: dict[str, list[tuple[str, str]]],
    is_packed: dict[str, bool],
    unions: dict[str, list[tuple[str, str]]] | None = None,
    defines: dict[str, int] | None = None,
) -> tuple[dict[str, tuple[str, int, int]], dict[str, tuple[int, int]]]:
    """Walks each struct's members in declaration order, simulating the
    compiler's alignment/padding rules. Any field whose type is a union
    (e.g. `union packet_data data;`) has that union's size computed
    C-style - the LARGEST member, not a sum - and represented as an
    opaque `{size}s` bytes blob in the format string, since a union's
    bytes don't have one fixed interpretation.

    Returns (struct_layouts, union_layouts):
      struct_layouts = {struct_name: (format_str, size_bytes, alignment_bytes)}
      union_layouts  = {union_name: (size_bytes, alignment_bytes)}
    format_str has no "<"/">" byte-order prefix - add one when you use it,
    e.g. "<" + struct_layouts["sensor_telemetry_t"][0].

    union_layouts is populated for every union reachable from a struct
    field, computed lazily the first time it's referenced - so it ends up
    complete as long as every union you care about (e.g. WIFI_PACKET_UNION
    is used somewhere) is actually used by at least one parsed struct.
    Relies on `structs` being in file-definition order (a plain dict
    preserves insertion order, and re.finditer yields matches in the order
    they appear in the file) so a nested `struct Foo bar;` field can look
    up Foo's already-computed layout - Foo must be defined earlier in the
    header than whatever embeds it, same as C itself requires.
    """
    defines = defines or {}
    unions = unions or {}
    known: dict[str, tuple[str, int, int]] = {}
    union_cache: dict[str, tuple[int, int]] = {}

    for name, members in structs.items():
        packed = is_packed.get(name, False)
        fmt_parts = []
        offset: int = 0
        max_align: int = 1

        for type_, var in members:
            m = _ARRAY_RE.match(var)
            var_name = m.group(1) if m else var
            array_len_raw = m.group(2) if m else None

            fmt_char, size, align = _resolve_type(
                type_, known, unions, union_cache, defines
            )
            count = _resolve_count(array_len_raw, defines, f"{name}.{var_name}")

            if not packed:
                pad = (-offset) % align
                if pad:
                    fmt_parts.append(f"{pad}x")
                    offset += pad
                max_align = max(max_align, align)

            total_bytes = size * count
            if fmt_char is None:
                # union field - opaque blob, decode further at read-time
                fmt_parts.append(f"{total_bytes}s")
            else:
                fmt_parts.append(f"{count}{fmt_char}" if count > 1 else fmt_char)
            offset += total_bytes

        if not packed:
            trailing_pad = (-offset) % max_align
            if trailing_pad:
                fmt_parts.append(f"{trailing_pad}x")
                offset += trailing_pad

        known[name] = ("".join(fmt_parts), offset, max_align if not packed else 1)

    # Also compute any union that no struct field referenced, so callers
    # can look up e.g. a top-level union's size even if nothing embeds it.
    for name in unions:
        if name not in union_cache:
            _union_layout(name, unions, known, union_cache, defines)

    return known, union_cache
