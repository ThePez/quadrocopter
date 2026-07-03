default_port := ""

# Ensures idf.py is actually on PATH before doing anything else.
# 'espidf' is a bash alias, so it only takes effect in the interactive
# shell you ran it in — just inherits that shell's environment when
# invoked from the same terminal, but can't invoke the alias itself
# (non-interactive shells don't expand aliases). Run `espidf` once in
# your terminal, then run `just ...` from that same terminal session.
_idf-check:
    @command -v idf.py >/dev/null 2>&1 || { \
        echo "idf.py not found on PATH. Run 'espidf' in this terminal first."; \
        exit 1; \
    }

build target: _idf-check
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{target}}" in
        drone|remote) chip=esp32 ;;
        bridge)       chip=esp32s3 ;;
        *) echo "Unknown target: {{target}} (expected drone, remote, or bridge)" >&2; exit 1 ;;
    esac

    # Only set-target if this is a fresh build dir (avoids unnecessary reconfigure)
    if [ ! -f "{{target}}/build/sdkconfig" ]; then
        idf.py -C {{target}} -B {{target}}/build set-target "$chip"
    fi

    idf.py -C {{target}} -B {{target}}/build build

flash target port=default_port: _idf-check
    idf.py -C {{target}} -B {{target}}/build \
        {{ if port != "" { "-p " + port } else { "" } }} \
        flash

monitor target port=default_port: _idf-check
    idf.py -C {{target}} -B {{target}}/build \
        {{ if port != "" { "-p " + port } else { "" } }} \
        monitor

clean target: _idf-check
    idf.py -C {{target}} -B {{target}}/build fullclean

all target port=default_port:
    just build {{target}}
    just flash {{target}} {{port}}