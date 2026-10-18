#include <dzuav/HbbDetector.hpp>
#include <dzuav/CellClassifier.hpp>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace dzuav;

static const char* kNames[] = {"mine","car","arrow","target_red","cell_label","target_blue","obstacle"};

int main(int argc, char** argv) {
  std::string dev   = argc > 1 ? argv[1] : "/dev/video0";
  std::string hbbp  = argc > 2 ? argv[2] : "/home/duzhong/dzuav/engines/hbb_detector_fp16.engine";
  std::string cellp = argc > 3 ? argv[3] : "/home/duzhong/dzuav/engines/cell_classifier_v4d_fp16.engine";
  int CW = 1280, CH = 720, FPS = 30;
  int WW = getenv("VIEW_W") ? atoi(getenv("VIEW_W")) : 1280;
  int WH = getenv("VIEW_H") ? atoi(getenv("VIEW_H")) : 1280;
  std::string pipe = "v4l2src device=" + dev + " ! image/jpeg,width=" + std::to_string(CW) +
                     ",height=" + std::to_string(CH) + ",framerate=" + std::to_string(FPS) +
                     "/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1";
  cv::VideoCapture cap(pipe, cv::CAP_GSTREAMER);
  if (!cap.isOpened()) { fprintf(stderr, "cannot open camera: %s\n", pipe.c_str()); return 3; }
  HbbDetector hbb(hbbp, .25f, .45f);
  CellClassifier cell(cellp);
  cv::Mat frame, canvas;
  long frames = 0;
  double t0 = (double)cv::getTickCount();
  while (true) {
    if (!cap.read(frame) || frame.empty()) { cv::waitKey(5); continue; }
    auto dets = hbb.infer(frame);
    for (auto& d : dets) {
      cv::Rect r = d.box & cv::Rect(0, 0, frame.cols, frame.rows);
      if (r.width < 2 || r.height < 2) continue;
      cv::Scalar c(0,255,255);
      if (d.type == ObjectClass::TargetBlue) c = cv::Scalar(255,128,0);
      else if (d.type == ObjectClass::TargetRed) c = cv::Scalar(0,0,255);
      else if (d.type == ObjectClass::CellLabel) c = cv::Scalar(0,200,0);
      cv::rectangle(frame, r, c, 2);
      char txt[96];
      snprintf(txt, sizeof(txt), "%s %.2f", kNames[(int)d.type], d.confidence);
      cv::putText(frame, txt, cv::Point(r.x, std::max(12, r.y-4)), cv::FONT_HERSHEY_SIMPLEX, 0.5, c, 1, 4);
      if (d.type == ObjectClass::CellLabel) {
        CellId id = cell.infer(frame(r));
        if (id.valid())
          cv::putText(frame, id.text(), cv::Point(r.x, r.y + r.height + 16), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
      }
    }
    frames++;
    double fps = (double)cv::getTickFrequency() * frames / ((double)cv::getTickCount() - t0);
    char top[160];
    snprintf(top, sizeof(top), "dzuav live %dx%d %.1ffps det=%zu", frame.cols, frame.rows, fps, dets.size());
    cv::putText(frame, top, cv::Point(10, 26), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);

    // letterbox into WW x WH canvas, keep aspect
    canvas = cv::Mat::zeros(WH, WW, CV_8UC3);
    double s = std::min((double)WW / frame.cols, (double)WH / frame.rows);
    int nw = (int)(frame.cols * s), nh = (int)(frame.rows * s);
    cv::Mat rs;
    cv::resize(frame, rs, cv::Size(nw, nh), 0, 0, s < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
    rs.copyTo(canvas(cv::Rect((WW - nw) / 2, (WH - nh) / 2, nw, nh)));

    cv::imshow("dzuav live", canvas);
    cv::moveWindow("dzuav live", 0, 0);
    if (frames % 30 == 0) cv::imwrite("/tmp/dzuav_view/latest.jpg", canvas);
    int k = cv::waitKey(1);
    if (k == 27 || k == 'q') break;
  }
  return 0;
}
