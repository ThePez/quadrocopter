import re
from pathlib import Path


def read_file(path: str) -> str:
    """Reads a C header and strips // and /* ... */ comments from it.

    Comments are stripped up front so none of the regexes below (here or
    in layout.py) need to worry about a struct/union/enum body containing
    something that only looks like C syntax inside a comment.
    """
    path = Path(path).resolve()
    text = path.read_text()
    text = re.sub(r"//.*", "", text)  # remove // style C comments
    text = re.sub(
        r"/\*.*?\*/", "", text, flags=re.DOTALL
    )  # remove /* ... */ style C comments
    return text


def _parse_body(body: str, list_: list) -> None:
    """Parses a flattened struct/union body into (type, var) pairs, appended
    in declaration order to `list_`.

    `body` must already be whitespace-collapsed to one line per statement
    with statements still separated by ";" (see parse_structs/parse_unions).
    Each statement's leading type token(s) are split off, then its
    comma-separated variable names are each paired with that same type -
    so `int a, b, c;` becomes three separate ("int", "a") / ("int", "b") /
    ("int", "c") entries. `var` may still carry a trailing "[N]" array
    suffix; that's resolved later by layout.py, not here.
    """
    for line in body.split(";"):
        line = line.strip()
        if not line:
            continue

        # Match each line of the body
        item = re.match(r"(struct\s\w+|union\s\w+|\w+)\s+(.*)", line)
        if item:
            type_ = item.group(1)
            vars = item.group(2)
            # Split incase mny variables are declared with the same type eg int a,b,c;
            for var in vars.split(","):
                list_.append((type_, var.strip()))


def parse_unions(file_path: str) -> dict:
    """Finds every `union <name> { ... };` block in a C header and parses
    each one's body into a {union_name: [(type, var), ...]} dict, in
    declaration order - mirroring parse_structs() but for unions.
    """
    unions = {}
    text = read_file(file_path)
    # Find all matches for union [name] {...};
    matches = re.finditer(r"union\s(\w*)\s*{(.*?)};", text, re.DOTALL)
    # Raise exception if no unions are found
    if not matches:
        raise ValueError("No C structs found in this file")

    for union in matches:
        name = union.group(1)
        # pre-format the body by removing newlines and other un-needed spaces
        body = " ".join(
            line.strip() for line in union.group(2).splitlines() if line.strip()
        )

        unions[name] = []
        _parse_body(body, unions[name])

    return unions


def parse_structs(file_path: str) -> tuple[dict, dict]:
    """Finds every `struct <name> { ... };` block in a C header (optionally
    `struct __packed <name> { ... };`) and parses each one's body into a
    {struct_name: [(type, var), ...]} dict, in declaration order - the
    order layout.py relies on to resolve nested struct/union fields.

    Returns (structs, is_packed), where is_packed maps struct_name -> bool
    for whether it was declared with the __packed attribute (no
    compiler-inserted padding between/after members).
    """
    structs = {}
    is_packed = {}
    text = read_file(file_path)
    # Find all matches for struct [name] {...};
    matches = re.finditer(r"struct\s(__packed\s)?(\w*)\s*{(.*?)};", text, re.DOTALL)

    # Raise exception if not structs found
    if not matches:
        raise ValueError("No C structs found in this file")

    for struct in matches:
        # Name of struct is first group
        name = struct.group(2)
        packed = struct.group(1) is not None
        is_packed[name] = packed
        # pre-format the body by removing newlines and other un-needed spaces
        body = " ".join(
            line.strip() for line in struct.group(3).splitlines() if line.strip()
        )

        structs[name] = []
        _parse_body(body, structs[name])

    return structs, is_packed


def parse_defines(header_text: str) -> dict[str, int]:
    """Pulls simple integer #define constants out of a C header, e.g.
    `#define OTA_DATA_CHUNK 128` -> {"OTA_DATA_CHUNK": 128}.

    A macro whose value is just another already-defined macro's name is
    resolved to that macro's value, e.g. `#define Y 4` then `#define X Y`
    -> {"Y": 4, "X": 4}. This only works forwards through the file, same as
    the C preprocessor - referencing a macro defined later, or an
    expression (`#define X (Y + 1)`), is not evaluated.
    """
    out: dict = {}
    for m in re.finditer(r"#define\s+(\w+)\s+(\w+|0[xX][0-9a-fA-F]+|\d+|0[0-9]+)", header_text):
        name, value = m.group(1), m.group(2)
        if value in out:
            out[name] = out[value]
        else:
            out[name] = int(value, 0)

    return out


def parse_enum(header_text: str, enum_name: str) -> dict[str, int]:
    """Parses a C `enum <enum_name> { ... }` block into {member_name: value}.

    Members without an explicit `= <value>` continue from the previous one,
    matching C's own enum numbering rules.
    """
    match = re.search(rf"enum\s+{enum_name}\s*{{(.*?)}}", header_text, re.DOTALL)
    if not match:
        raise ValueError(f"enum {enum_name} not found")
    body = re.sub(r"//.*", "", match.group(1))
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.DOTALL)

    values = {}
    next_value = 0
    for entry in body.split(","):
        entry = entry.strip()
        if not entry:
            continue
        name, _, raw_value = entry.partition("=")
        name = name.strip()
        if raw_value.strip():
            next_value = int(raw_value.strip(), 0)
        values[name] = next_value
        next_value += 1
    return values
