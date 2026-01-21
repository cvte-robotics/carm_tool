#!/bin/bash
SCRIPT_DIR="$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)"
LIB1="$SCRIPT_DIR/install/lib"
case ":$LD_LIBRARY_PATH:" in
  *":$LIB1:"*) ;;
  *) export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$LIB1" ;;
esac
export carm_tool_DIR=$SCRIPT_DIR
echo "find_package DIR: $carm_tool_DIR"
echo "[setup.bash] done!"

