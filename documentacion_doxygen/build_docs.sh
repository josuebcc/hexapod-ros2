#!/usr/bin/env bash
#
# Genera la documentación Doxygen del firmware del hexápodo.
#
#   ./build_docs.sh          genera la web en docs/html/
#   ./build_docs.sh --open   la genera y la abre en el navegador
#
set -euo pipefail

cd "$(dirname "$0")"

if ! command -v doxygen >/dev/null 2>&1; then
    echo "error: doxygen no está instalado (sudo apt install doxygen)" >&2
    exit 1
fi

if ! command -v dot >/dev/null 2>&1; then
    echo "aviso: graphviz no está instalado; los diagramas no se generarán" >&2
    echo "       (sudo apt install graphviz)" >&2
fi

rm -rf docs/html
mkdir -p docs
doxygen Doxyfile

index="$(pwd)/docs/html/index.html"
log="$(pwd)/docs/doxygen-warnings.log"

echo
echo "Documentación generada:"
echo "  $index"

if [[ -s "$log" ]]; then
    echo "  $(grep -c 'warning:' "$log") avisos en $log"
fi

if [[ "${1:-}" == "--open" ]]; then
    xdg-open "$index" >/dev/null 2>&1 &
fi
