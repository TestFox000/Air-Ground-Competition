#!/bin/bash
echo ---OUTS; ls -la /tmp/verify_old /tmp/verify_new /tmp/v88_old /tmp/v88_new
echo ---MATRIX; find ~/dzuav_deploy ~/ag_test ~/dzuav -maxdepth 5 -type f \( -name "*matrix*.json" -o -name "*homograph*" -o -name "*calib*.json" \) 2>/dev/null | head -10
echo ---FLIGHT_IMAGES; find ~/ag_flight_logs ~/drone_logs ~/drone_logs_20261015 -maxdepth 3 -type f \( -iname "*.jpg" -o -iname "*.png" \) 2>/dev/null | head -10
echo ---COUNT_IMG; find ~/ag_flight_logs -type f -iname "*.jpg" 2>/dev/null | wc -l
echo ---TAR; ls -la ~/drone_flight_logs_export_20261017.tar.gz 2>/dev/null
echo ---CONFIG; ls ~/dzuav_deploy/air_ground/src/ag_integrate/config 2>/dev/null
