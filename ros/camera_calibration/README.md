# Camera calibration

*Build container*: `docker-compose -f docker-compose-camera-calibration.yml build`

*Run container*: `./calibrate_camera.sh`

After calibration, the results are saved in `/tmp` directory inside the container.
Use `get_camera_settings.sh` script to retrieve the results from the container.
It will save `.yaml` file to the `../camera_settings` directory and `calibrationdata.tar.gz` archive to the parrent directory (`ros` directory).
