#ifndef TexSynHandler_H

#include <memory>
#include <string>
#include <vector>
#include <cv.h>

class Model;
class DetailSynthesis;
class AppearanceModel;
class GLActor;

class TexSynHandler
{
public:
  TexSynHandler();
  ~TexSynHandler();

  void setSynthesisModel(std::shared_ptr<Model> model);
  void runD1Synthesis(std::string app_mod_file);
  std::string runD0Synthesis(std::string app_mod_file);
  std::string applyD1Displacement(cv::Mat& mask);

  std::vector<GLActor>& getGLActors() { return actors; };


  std::shared_ptr<AppearanceModel> get_syn_app_mod(){ return this->syn_app_mod; };
private:
  std::shared_ptr<Model>           synthesis_model;
  std::shared_ptr<AppearanceModel> syn_app_mod;

  std::shared_ptr<DetailSynthesis> detail_synthesis;

  std::vector<GLActor> actors;

private:
  TexSynHandler(const TexSynHandler&);
  void operator = (const TexSynHandler&);
};

#endif // !TexSynHandler_H
