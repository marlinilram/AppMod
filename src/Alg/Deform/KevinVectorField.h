#ifndef KevinVectorField_H
#define KevinVectorField_H

#include "BasicHeader.h"
#include <memory>
#include "cholmod_matrix.h"
#include "cholmod_vector.h"
#include "PolygonMesh.h"
#include "Model.h"
#include "GLActor.h"
#include "Vector3.h"
#include <string>

#define BN (1.0e8)
#define SQBN (1.0e4)

typedef Cholmod_matrix<double>			CMatrix;
typedef Cholmod_vector<double>			CVector;
typedef enum _Crest_type{CONVEX=0, CONCAVE} Crest_type;
typedef std::vector<std::vector<std::pair<int,int>>> Branch;
typedef std::map<int,std::pair<Vector3,short>>	MAP_IVS;	// map int to Vector3&short

template <typename FT>
class CLI	// crest line information
{
public:
	class CSeg
	{
	public:
		CSeg() {v[0]=v[1]=fi=0;};
		CSeg(int a, int b, int f) {v[0]=a; v[1]=b; fi=f; l=0.0;};
		~CSeg() {};
		void set(int a, int b, int f) {v[0]=a; v[1]=b; fi=f;};
		CSeg& operator=(const CSeg& in) {v[0]=in.v[0]; v[1]=in.v[1]; fi=in.fi; return *this;};
		CSeg& operator&(const CSeg& in) {v[0]=in.v[1]; v[1]=in.v[0]; fi=in.fi; return *this;};
		int v[2], fi;
		FT l;
	};
	typedef std::vector<Vector3>	VV;
	typedef std::vector<CSeg>		SV;

	SV seg;
	VV dir;
	int tag;	// 0:filtered 1:unfiltered
	Crest_type type;
	FT l;	// length
	FT r;	// ridgeness
	FT s;	// sphericalness
	FT c;	// cyclideness
	CLI() {tag=1; r=0.0; s=0.0; c=0.0; l=0.0; type=CONVEX;};
	~CLI() {};
};

class KevinVectorField
{
public:
  KevinVectorField();
  ~KevinVectorField();

  void init(std::shared_ptr<Model> model);

  void load_crestlines();
  int read_crestlines(std::ifstream &stream, Crest_type type);
  void clean_lines(void);
  void make_seg_consistent(void);
  void find_branch(Branch &Branch);
  void trace_seg_branch(CLI<FT> &NewCL, const CLI<FT> &CL, Branch &branch, int p, int s);
  void del_crestlines(std::set<int> &dels, Branch &branch);
  void make_seg_sequent(void);
  void compute_smooth_dir(void);
  void weld_at_branches(Branch &branch);
  int merge_two_lines(CLI<FT> &NewCL, const CLI<FT> &CL1, bool bL1R, const CLI<FT> &CL2, bool bL2R, bool bCP);
  void compute_line_length();
  void get_crest_range(void);
  void exportCCL();
  int readCCL(std::ifstream &stream);

  void compute_site_vectors(void);
  FT sqdPnt2LineSeg(const Vector3 &ls, const Vector3 &le, const Vector3 &p);
  void compute_s_hvf(void);
  void set_s_hvf_rhs_vec(CVector *tvRHSV, int dim);
  void set_s_hvf_lap_uniform_PNT(CMatrix *tmAssLM);
  void project_harmonic_vector_field(Vector3 *hvf, FT low, FT high);
  inline bool IsZeroL(FT f){ return fabs(f) < 0.000001f;};

  void runCrestCode();
  void getDrawableActors(std::vector<GLActor>& actors);

private:
  std::vector<GLActor> actors;

  std::shared_ptr<Model> shape_model;

  std::vector<CLI<FT>>	m_pCLInfo;
  std::vector<Vector3>	m_Points;
  MAP_IVS					m_hvf_sites;
  MAP_IVS         m_pMesh_hvf_site;
  cholmod_factor				*m_s_hvf_F;
  Vector3						*m_s_vf;
  Branch					m_Branch;

  bool					m_bSegConsist;
	bool					m_bSegSequent;
	bool					m_bCompLen;
  bool						m_is_s_hvf_utd;

  FT							m_s_hvm_max;		// maximum magnitude
	FT							m_s_hvm_min;		// minimum magnitude
  FT					m_dMinR;		// minimum ridgeness
	FT					m_dMaxR;		// maximum ridgeness
	//FT					m_dTR;			// threshold of ridgeness
	FT					m_dMinS;		// minimum sphericalness
	FT					m_dMaxS;		// maximum sphericalness
	//FT					m_dTS;			// threshold of sphericalness
	FT					m_dMinC;		// minimum cyclideness
	FT					m_dMaxC;		// maximum cyclideness
	//FT					m_dTC;			// threshold of cyclideness
	FT					m_dMinL;		// minimum length
	FT					m_dMaxL;		// maximum length
	//FT					m_dTL;			// threshold of length
};



#endif

