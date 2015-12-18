#ifndef FeatureGuided_H
#define FeatureGuided_H

#include <cv.h>
#include <highgui.h>
#include <memory>

#include "BasicDataType.h"
#include "kdtree.h" // to make life easier...
#include "BasicHeader.h"

class Model;
class FeatureLine;
class ScalarField;
class KDTreeWrapper;
class tele2d;
// suppose the HIST contains 8 + 1 + 2 elements
// 8 for directions, 1 for scalar and 2 for pos (x, y)
typedef             std::vector<double>    HIST;
typedef std::vector<std::vector<double> >  HISTS;

class FeatureGuided
{
public:
  FeatureGuided(std::shared_ptr<Model> source_model, std::string targetFile);
  FeatureGuided();
  virtual ~FeatureGuided();

  void initTargetImage(std::string targetFile);
  void initRegister();
  void initSrcRegister();
  void initTarRegister(int type = 0);
  void updateSourceVectorField();
  void updateScalarField();
  void updateSourceField(int update_type = 0);
  void updateDistSField();
  std::shared_ptr<KDTreeWrapper> getSourceKDTree();
  std::shared_ptr<KDTreeWrapper> getTargetKDTree();
  std::map<int, std::pair<int, int> >& getSourceKDTreeMapper();
  std::map<int, std::pair<int, int> >& getTargetKDTreeMapper();
  CURVES& getSourceCurves();
  CURVES& getTargetCurves();
  inline double getCurveScale() { return curve_scale; };
  inline double2 getCurveTranslate() { return curve_translate; };

  inline std::shared_ptr<tele2d> GetTeleRegister() { return source_tele_register; };
  inline std::shared_ptr<tele2d> GetTargetTeleRegister() { return target_tele_register; };
  inline std::shared_ptr<ScalarField> getSourceScalarField() { return source_scalar_field; };
  inline std::shared_ptr<ScalarField> getTargetScalarField() { return target_scalar_field; };
  void NormalizedTargetCurves(CURVES& curves);
  void NormalizedSourceCurves(CURVES& curves);
  void NormalizedPts(double2& pt);
  void setNormalizePara();
  void OptimizeConnection();
  double MatchScoreToVectorField(std::vector<double2>& curve);
  void BuildDispMap(const cv::Mat& source, kdtree::KDTreeArray& KDTree_data);
  void BuildSourceEdgeKDTree();
  void BuildTargetEdgeKDTree();
  void BuildEdgeKDTree(CURVES& curves, std::map<int, std::pair<int, int> >& id_mapper, std::shared_ptr<KDTreeWrapper> kdTree);
  void GetSourceNormalizePara(double2& translate, double& scale);
  void GetFittedCurves(CURVES& curves);
  void CalculateHists(
    HISTS& hists,
    CURVES& curves, double radius, tele2d* tele);
  void SearchRadius(
    std::vector<double>& hist,
    double2 center, double r, std::vector<int2>& area,
    std::vector<double2>& vector_field, int resolution);
  void FindHistMatchCrsp(CURVES &curves);
  void GetCrspPair(CURVES& curves);
  void GetUserCrspPair(CURVES& curves, float sample_density);
  void BuildClosestPtPair();
  void GetCurrentCrspList(std::vector<std::pair<int, double2> >& crsp_list);
  void setUserCrspPair(double start[2], double end[2]);
  void BuildClosestPtPair(CURVES& curves, std::map<int, std::pair<Vector2f, Vector2f> >& data_crsp); // data_crsp stores corresponding vid-pixel pair

  void ExtractCurves(const cv::Mat& source, CURVES& curves);
  void ExtractSrcCurves(const cv::Mat& source, CURVES& curves);
  void SearchCurve(const cv::Mat& source,
    int cur_row, int cur_col,
    std::vector<std::vector<bool>>& visited_table,
    std::vector<double2>& curve);
  void AnalyzeTargetRelationship();

  inline std::map<std::pair<int, int>, int>& getSrcVidMapper() { return src_vid_mapper; };
  inline std::map<int, std::pair<int, int> >& getSrcRevVidMapper() { return src_rev_vid_mapp; };
  void getNormalizedProjPt(const int vid, double2& proj_pos);

  void updateUserMarkedCurves();
  void locateMarkedCurves();
  std::map<int, int>& getVisibleGlobalMapper();
  std::map<int, std::vector<int>>& getGlobalVisibleMapper();
  std::vector<double>& getEdgesAverageSpSl();
  void deleteTargetCurves(std::vector<int>& deleted_tags);
  void computeAverageTargetCurvesDir();

public:

  // user defined feature line
  std::shared_ptr<FeatureLine> source_vector_field_lines;
  std::shared_ptr<FeatureLine> target_vector_field_lines;

  std::vector<std::pair<int, int> > src_crsp_list;
  std::vector<std::pair<int, int> > tar_crsp_list;
  std::map<int, double2> user_correct_crsp_map;

  int user_constrained_src_v; // the index of v in model
  double2 user_constrained_tar_p; // target screen position

  std::vector<int> user_marked_crsp;  // source & target curves user has marked 
  std::set<std::pair<int, int> > global_user_marked_crsp; // all source & target curves user has marked for each stage (source curves are stored using the original crest lines id)
  std::vector<int> marked_source_curves; // all source curves user has marked for each stage (source curves are stored using the source curves id)
  std::vector<int> marked_target_curves; // all target curves user has marked for each stage
  //std::set<std::pair<int, int> > user_define_curve_crsp;
  double sample_rate;
  std::vector<double> src_sample_rate;
  double average_sourcePts_interval;
  std::vector<std::vector<Vector2f>> sampled_target_curves_average_dir;

private:
  
  friend class ProjICP;
  friend class LargeFeatureReg;
  friend class ScalarField;
  friend class LargeFeatureCrsp;

  std::shared_ptr<Model> source_model;
  cv::Mat target_img;
  cv::Mat target_edge_saliency;
  std::vector<std::vector<double2>> target_edges_sp_len; // attribute storing the length to each end of this edge for each edge sample point
  std::vector<std::vector<double> > target_edges_sp_sl;
  std::vector<double> target_edges_average_sp_sl;

  CURVES source_curves;
  CURVES target_curves;
  double2 curve_translate;
  double curve_scale;
  float edge_threshold; // threshold for edge detection
  std::map<std::pair<int, int>, int> src_vid_mapper; // map from curve id to vertex id
  std::map<int, std::pair<int, int> > src_rev_vid_mapp; // map from vertex id to curve id
  std::vector<std::set<int> > tar_relationship;
  std::vector<Vector2f>       tar_avg_direction;
  std::vector<Vector2f>       src_avg_direction;
  
  std::shared_ptr<KDTreeWrapper> source_KDTree;
  std::shared_ptr<KDTreeWrapper> target_KDTree;
  std::map<int, std::pair<int, int> > kdtree_id_mapper; // map from target kdtree id to curves id
  std::map<int, std::pair<int, int> > kdtree_id_mapper_source;
  //std::map<int, std::pair<int, int> > kdtree_id_mapper_target;

  std::shared_ptr<tele2d> source_tele_register;
  std::shared_ptr<tele2d> target_tele_register;

  std::shared_ptr<ScalarField> source_scalar_field;
  std::shared_ptr<ScalarField> target_scalar_field;
private:
  FeatureGuided(const FeatureGuided&);
  void operator = (const FeatureGuided&);
};

#endif