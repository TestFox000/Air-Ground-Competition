#include <dzuav/HbbDetector.hpp>
#include <dzuav/CellClassifier.hpp>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <algorithm>

using namespace dzuav;

static const char* kNames[] = {"mine","car","arrow","target_red","cell_label","target_blue","obstacle"};

int main(int argc, char** argv) {
  std::string dev   = argc > 1 ? argv[1] : "/dev/video0";
  std::string hbbp  = argc > 2 ? argv[2] : "/home/duzhong/dzuav/engines/hbb_detector_fp16.engine";
  std::string cellp = argc > 3 ? argv[3] : "/home/duzhong/dzuav/engines/cell_classifier_v4d_fp16.engine";
  int CW = getenv("CAP_W") ? atoi(getenv("CAP_W")) : 1920;
  int CH = getenv("CAP_H") ? atoi(getenv("CAP_H")) : 1200;
  int WW = getenv("VIEW_W") ? atoi(getenv("VIEW_W")) : 1280;
  int WH = getenv("VIEW_H") ? atoi(getenv("VIEW_H")) : 1280;
  int FPS = getenv("CAP_FPS") ? atoi(getenv("CAP_FPS")) : 30;
  std::string pipe = "v4l2src device=" + dev + " ! image/jpeg,width=" + std::to_string(CW) +
                     ",height=" + std::to_string(CH) + ",framerate=" + std::to_string(FPS) +
                     "/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1";
  cv::VideoCapture cap(pipe, cv::CAP_GSTREAMER);
  if (!cap.isOpened()) { fprintf(stderr, "cannot open camera: %s\n", pipe.c_str()); return 3; }
  HbbDetector hbb(hbbp, .25f, .45f);
  CellClassifier cell(cellp);
  cv::Mat frame, sq;
  long frames = 0;
  double t0 = (double)cv::getTickCount();
  while (true) {
    if (!cap.read(frame) || frame.empty()) { cv::waitKey(5); continue; }
    int side = std::min(frame.cols, frame.rows);
    int x0 = (frame.cols - side) / 2, y0 = (frame.rows - side) / 2;
    cv::Mat crop = frame(cv::Rect(x0, y0, side, side));
    cv::resize(crop, sq, cv::Size(WW, WH), 0, 0, cv::INTER_LINEAR);
    auto dets = hbb.infer(sq);
    for (auto& d : dets) {
      cv::Rect r = d.box & cv::Rect(0, 0, sq.cols, sq.rows);
      if (r.width < 2 || r.height < 2) continue;
      cv::Scalar c(0,255,255);
      if (d.type == ObjectClass::TargetBlue) c = cv::Scalar(255,128,0);
      else if (d.type == ObjectClass::TargetRed) c = cv::Scalar(0,0,255);
      else if (d.type == ObjectClass::CellLabel) c = cv::Scalar(0,200,0);
      cv::rectangle(sq, r, c, 2);
      char txt[96];
      snprintf(txt, sizeof(txt), "%s %.2f", kNames[(int)d.type], d.confidence);
      cv::putText(sq, txt, cv::Point(r.x, std::max(12, r.y-4)), cv::FONT_HERSHEY_SIMPLEX, 0.5, c, 1, 4);
      if (d.type == ObjectClass::CellLabel) {
        CellId id = cell.infer(sq(r));
        if (id.valid())
          cv::putText(sq, id.text(), cv::Point(r.x, r.y + r.height + 16), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
      }
    }
    frames++;
    double fps = (double)cv::getTickFrequency() * frames / ((double)cv::getTickCount() - t0);
    char top[192];
    snprintf(top, sizeof(top), "dzuav live %dx%d crop %dx%d %.1ffps det=%zu", sq.cols, sq.rows, side, side, fps, dets.size());
    cv::putText(sq, top, cv::Point(10, 26), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
    cv::imshow("dzuav live", sq);
    cv::moveWindow("dzuav live", 0, 0);
    if (frames % 30 == 0) cv::imwrite("/tmp/dzuav_view/latest_sq.jpg", sq);
    int k = cv::waitKey(1);
    if (k == 27 || k == 'q') break;
  }
  return 0;
}
