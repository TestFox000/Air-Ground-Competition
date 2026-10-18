#include <opencv2/opencv.hpp>
#include <cstdio>
int main() {
  cv::Mat img(1280, 1280, CV_8UC3, cv::Scalar(100, 120, 140));
  int N = 20;
  double t0 = cv::getTickCount();
  for (int i = 0; i < N; i++) {
    cv::Mat r;
    cv::resize(img, r, cv::Size(1280, 1280));
    cv::Mat blob = cv::dnn::blobFromImage(r, 1/255.f, cv::Size(1280, 1280), {}, true, false);
  }
  double ms = (cv::getTickCount() - t0) / cv::getTickFrequency() * 1000 / N;
  printf("blobFromImage1280 avg %.1f ms\n", ms);
  return 0;
}
