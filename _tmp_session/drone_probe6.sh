#!/bin/bash
echo ---RUN_SCRIPT; cat ~/dzuav_deploy/air_ground/tools/orin_run_dzuav.sh
echo ---ENGINES_DIR; ls -la ~/dzuav_deploy/air_ground/engines 2>/dev/null
echo ---AIRGROUND; ls -la ~/dzuav_deploy/air_ground | head -25
