#include <dzuav/HbbDetector.hpp>
#include <dzuav/CellClassifier.hpp>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <algorithm>

using namespace dzuav;

static const char* kNames[] = {"mine","car","arrow","target_red","cell_label","target_blue","obstacle"};

int main(int argc, char** argv) {
  std::string dev   = argc > 1 ? argv[1] : "/dev/video0";
  std::string hbbp  = argc > 2 ? argv[2] : "/home/duzhong/dzuav/engines/hbb_detector_fp16.engine";
  std::string cellp = argc > 3 ? argv[3] : "/home/duzhong/dzuav/engines/cell_classifier_v4d_fp16.engine";
  int CW   = getenv("CAP_W")  ? atoi(getenv("CAP_W"))  : 1280;
  int CH   = getenv("CAP_H")  ? atoi(getenv("CAP_H"))  : 720;
  int CFPS = getenv("CAP_FPS")? atoi(getenv("CAP_FPS")): 30;
  int DW   = getenv("DISP_W") ? atoi(getenv("DISP_W")) : 1280;
  int DH   = getenv("DISP_H") ? atoi(getenv("DISP_H")) : 720;
  if (DW < 160) DW = 160; if (DW > 1920) DW = 1920;
  if (DH < 90)  DH = 90;  if (DH > 1080) DH = 1080;

  std::string pipe = "v4l2src device=" + dev + " ! image/jpeg,width=" + std::to_string(CW) +
                     ",height=" + std::to_string(CH) + ",framerate=" + std::to_string(CFPS) +
                     "/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1";
  cv::VideoCapture cap(pipe, cv::CAP_GSTREAMER);
  if (!cap.isOpened()) { fprintf(stderr, "cannot open camera: %s\n", pipe.c_str()); return 3; }
  HbbDetector hbb(hbbp, .25f, .45f);
  CellClassifier cell(cellp);

  cv::Mat frame, infer_img;
  long frames = 0;
  double t0 = (double)cv::getTickCount();
  bool moved = false;

  while (true) {
    if (!cap.read(frame) || frame.empty()) { cv::waitKey(5); continue; }

    // ---- full-frame letterbox: keep every camera pixel, pad to 1280x1280 ----
    float s = std::min(1280.0f / (float)frame.cols, 1280.0f / (float)frame.rows);
    cv::Mat r;
    cv::resize(frame, r, cv::Size(), s, s, cv::INTER_LINEAR);
    int top = (1280 - r.rows) / 2, bot = 1280 - r.rows - top;
    int lef = (1280 - r.cols) / 2, rig = 1280 - r.cols - lef;
    cv::copyMakeBorder(r, infer_img, top, bot, lef, rig, cv::BORDER_CONSTANT, cv::Scalar(114,114,114));

    auto dets = hbb.infer(infer_img);

    // ---- draw boxes back on the ORIGINAL full frame (nothing cropped) ----
    cv::Mat disp = frame.clone();
    for (auto& d : dets) {
      cv::Rect br = d.box & cv::Rect(0, 0, 1280, 1280);
      if (br.width < 2 || br.height < 2) continue;
      float x0 = (br.x - lef) / s, y0 = (br.y - top) / s;
      float x1 = (br.x + br.width - lef) / s, y1 = (br.y + br.height - top) / s;
      cv::Rect rr(cvRound(x0), cvRound(y0), cvRound(x1 - x0), cvRound(y1 - y0));
      rr &= cv::Rect(0, 0, frame.cols, frame.rows);
      if (rr.width < 1 || rr.height < 1) continue;
      cv::Scalar c(0,255,255);
      if (d.type == ObjectClass::TargetBlue) c = cv::Scalar(255,128,0);
      else if (d.type == ObjectClass::TargetRed) c = cv::Scalar(0,0,255);
      else if (d.type == ObjectClass::CellLabel) c = cv::Scalar(0,200,0);
      cv::rectangle(disp, rr, c, 2);
      char txt[96];
      snprintf(txt, sizeof(txt), "%s %.2f", kNames[(int)d.type], d.confidence);
      cv::putText(disp, txt, cv::Point(rr.x, std::max(14, rr.y-4)), cv::FONT_HERSHEY_SIMPLEX, 0.55, c, 2, 4);
      if (d.type == ObjectClass::CellLabel) {
        CellId id = cell.infer(infer_img(br));
        if (id.valid())
          cv::putText(disp, id.text(), cv::Point(rr.x, std::min(frame.rows-4, rr.y + rr.height + 16)), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
      }
    }

    cv::Mat show;
    cv::resize(disp, show, cv::Size(DW, DH), 0, 0, cv::INTER_AREA);

    frames++;
    double fps = (double)cv::getTickFrequency() * frames / ((double)cv::getTickCount() - t0);
    char hud[256];
    snprintf(hud, sizeof(hud), "dzuav %dx%d FULL-frame (letterbox 1280) %.1ffps det=%zu",
             DW, DH, fps, dets.size());
    cv::putText(show, hud, cv::Point(8, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
    if (frames % 30 == 0) {
      cv::imwrite("/tmp/dzuav_view/latest_full.jpg", show);
      printf("[STAT] frames=%ld fps=%.2f cap=%dx%d disp=%dx%d det=%zu\n", frames, fps, CW, CH, DW, DH, dets.size());
      fflush(stdout);
    }
    cv::imshow("dzuav live", show);
    if (!moved) { cv::moveWindow("dzuav live", 0, 0); moved = true; }
    int kk = cv::waitKey(1);
    if (kk == 27 || kk == 'q') break;
  }
  return 0;
}
