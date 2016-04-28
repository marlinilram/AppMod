#ifndef TexSynHandler_H

#include <memory>
#include <string>

class Model;
class DetailSynthesis;
class AppearanceModel;

class TexSynHandler
{
public:
  TexSynHandler();
  ~TexSynHandler();

  void setSynthesisModel(std::shared_ptr<Model> model);
  void runD1Synthesis(std::string app_mod_file);
  void runD0Synthesis(std::string app_mod_file);

private:
  std::shared_ptr<Model>           synthesis_model;
  std::shared_ptr<AppearanceModel> syn_app_mod;

  std::shared_ptr<DetailSynthesis> detail_synthesis;

private:
  TexSynHandler(const TexSynHandler&);
  void operator = (const TexSynHandler&);
};

#endif // !TexSynHandler_H
