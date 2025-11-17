SmartFuelApp 사용법

SmartFuelApp으로 들어가서
```
flutter run 
```
-d chrome(옵션)

명령어 실행
top/xyz_project/xyz_1st_project/SmartFuelApp$ flutter run -d chrome
Connected devices:
Linux (desktop) • linux  • linux-x64      • Ubuntu 24.04.3 LTS
6.14.0-35-generic
Chrome (web)    • chrome • web-javascript • Google Chrome
142.0.7444.162
[1]: Linux (linux)
[2]: Chrome (chrome)
Please choose one (or "q" to quit): 


이 화면에서 linux, chrome 할지 선택

서버 틀어야됨!
/home/deepet/Desktop/xyz_project/xyz_1st_project/SmartFuelApp/server
여기서 

```bash
uvicorn server:app --host 0.0.0.0 --port 8000 --reload
```

https://aistudio.google.com/app/apikey
이 사이트에서 api키 받아서 
/home/deepet/Desktop/xyz_project/xyz_1st_project/SmartFuelApp/lib/services/llm_service.dart
여기에서
  static const String _apiKey = '여기에 넣으면 됨!';
-> 이런식으로
  static const String _apiKey = 'AIzaSyAR_NzvrTRQnVXGWEw0rOHC-RWPelP-BhI';



flutter run

uvicorn server:app --host 0.0.0.0 --port 8000 --reload

첫번째거는 smartfuelapp 디렉토리 내에서 하면 됨

두번째거는 smartfuelapp/server  디렉토리 내에서 하면 됨

https://aistudio.google.com/app/apikey



전체 코드 돌리는 명령어 8개
uvicorn server:app --host 0.0.0.0 --port 8000 --reload
flutter run 

ros2 run dsr_example webcam_streamer
ros2 run dsr_example webcam_manager_ros
ros2 run dsr_example realsense_streamer
ros2 run dsr_example realsense_manager_ros
ros2 run dsr_example ocr_manager_ros
ros2 run dsr_example ocr_streamer

