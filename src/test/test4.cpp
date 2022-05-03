#include "../image.h"
#include "../utils.h"
#include "../matrix.h"

#include <string>

using namespace std;


void test_structure()
  {
  Image im = load_image("data/dogbw.png");
  Image s = structure_matrix(im, 2, true);
  s.feature_normalize_total();
  save_png(s, "output/structure");
  Image gt = load_image("data/structure.png");
  
  TEST(same_image(s, gt));
  }

void test_cornerness()
  {
  Image im = load_image("data/dogbw.png");
  Image s = structure_matrix(im, 2, true);
  Image c = cornerness_response(s,0);
  c.feature_normalize_total();
  save_png(c, "output/response");
  Image gt = load_image("data/response.png");
  TEST(same_image(c, gt));
  }


void run_tests()
  {
  test_structure();
  test_cornerness();
  
  printf("%d tests, %d passed, %d failed\n", tests_total, tests_total-tests_fail, tests_fail);
  }

int main(int argc, char **argv)
  {
  
  //test_matrix();
  
  run_tests();

  srand (time(NULL));

  Image a = load_image("pano/rainier/Rainier1.png");
  Image b = load_image("pano/rainier/Rainier2.png");

  float cor_thresh = 0.1;
  int desc_window = 7;
  int nms_window = 3;
  int ransac_iters = 10000;
  float ransac_inlier_thresh = 2;
  Image corners=detect_and_draw_corners(a, 2, cor_thresh, desc_window, nms_window, 0);
  save_image(corners, "output/corners");

  vector<Descriptor> ad=harris_corner_detector(a, 2, cor_thresh, desc_window, nms_window, 0);
  vector<Descriptor> bd=harris_corner_detector(b, 2, cor_thresh, desc_window, nms_window, 0);
  
  vector<Match> match=match_descriptors(ad,bd);
  Image inliers=draw_inliers(a,b,RANSAC(match,ransac_inlier_thresh,ransac_iters,50),match,ransac_inlier_thresh);
  
  Image m = find_and_draw_matches(a, b, 2, cor_thresh, desc_window, nms_window, 0);
  Image pan=panorama_image(a,b,2,0,cor_thresh,desc_window,nms_window,ransac_inlier_thresh,ransac_iters,50,0.5);

  save_image(inliers, "output/inliers");
  save_image(m, "output/matches");
  save_image(pan, "output/easy_panorama");
  
//  save_image(panorama_image(cylindrical_project(a,500),cylindrical_project (b,500),2,0,0.3,7,3,5,1000,50,0.5), "output/easy_panorama_cyl");
// save_image(panorama_image(spherical_project(a,500),spherical_project(b,500),2,0,0.3,7,3,5,1000,50,0.5), "output/easy_panorama_sphere");
  
  
  return 0;
  }
