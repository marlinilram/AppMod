#include "KevinVectorField.h"
#include "fstream"
#include <list>
#include "Bound.h"

#define FT_MAX DBL_MAX
using namespace LG;

KevinVectorField::KevinVectorField()
{

}

KevinVectorField::~KevinVectorField()
{

}

void KevinVectorField::init(std::shared_ptr<Model> model)
{
  shape_model = model;
  actors.push_back(GLActor(ML_POINT, 3.0f));
  actors.push_back(GLActor(ML_MESH, 1.0f));
  actors.push_back(GLActor(ML_LINE, 2.0f));
  actors[0].clearElement();
  actors[1].clearElement();
  actors[2].clearElement();
  m_s_hvf_F = NULL;
  m_s_vf = NULL;
}

void KevinVectorField::runCrestCode()
{
  FILE *fp,*fp_ply2;
  fp = fopen((shape_model->getOutputPath() + "/input.txt").c_str(),"w+");
  fp_ply2 = fopen((shape_model->getOutputPath() +"/model.ply2").c_str(),"w+");
  size_t vertexNum = (shape_model->getShapeVertexList()).size();
  size_t faceNum = (shape_model->getShapeFaceList()).size();
  int neighborhoodSize = 1;                            //need to be reset!
  fprintf(fp,"%d\n",vertexNum / 3);
  fprintf(fp,"%d\n",faceNum / 3);
  fprintf(fp,"%d\n",neighborhoodSize);
  fprintf(fp,"%d\n",1);

  fprintf(fp_ply2,"%d\n",vertexNum / 3);
  fprintf(fp_ply2,"%d\n",faceNum / 3);

  for(size_t i = 0 ; i < vertexNum ; i ++)
  {
    fprintf(fp,"%f\n",(shape_model->getShapeVertexList())[i]);
    fprintf(fp_ply2,"%f\n",(shape_model->getShapeVertexList())[i]);
  }
  for(size_t j = 0 ; j < faceNum ; j ++)
  {
    fprintf(fp,"%d\n",(shape_model->getShapeFaceList())[j]);
    if(j % 3 == 0)
    {
      fprintf(fp_ply2,"%d\n",3);
    }
    fprintf(fp_ply2,"%d\n",(shape_model->getShapeFaceList())[j]);
  }
  fclose(fp);
  fclose(fp_ply2);
}

int KevinVectorField::read_crestlines(std::ifstream &stream, Crest_type type)
{
	std::string buf;
	int num_points, num_segs, num_lines;
	stream >> num_points;
	if (num_points<0 || num_points>=(shape_model->getShapeVertexList().size() / 3)) {
		return -1;
	}
	stream >> num_segs;
	if (num_segs < 0) {
		return -1;
	}
	stream >> num_lines;
	if (num_lines < 0) {
		return -1;
	}
	if (num_lines == 0) {
		return 0;
	}
	int ls = m_pCLInfo.size();
	int ps = m_Points.size();
	m_pCLInfo.resize(m_pCLInfo.size() + num_lines);
	m_Points.resize(m_Points.size() + num_points);
	m_Branch.resize(m_Branch.size() + num_points);
	for (int i=0; i<num_points; i++)
	{
		m_Branch[ps+i].resize(1);
		stream >> m_Points[ps+i].x >> m_Points[ps+i].y >> m_Points[ps+i].z >> m_Branch[ps+i][0].first;
		m_Branch[ps+i][0].first += ls;
		m_Branch[ps+i][0].second = 0;
	}
	for (int i=0; i<num_lines; i++)
	{
		stream >> m_pCLInfo[ls+i].r >> m_pCLInfo[ls+i].s >> m_pCLInfo[ls+i].c;
		m_pCLInfo[ls+i].type = type;
	}
	int ev1, ev2, fi;
	for (int i=0; i<num_segs; i++)
	{
		stream >> ev1 >> ev2 >> fi;
		if (m_Branch[ps+ev1][0].first != m_Branch[ps+ev2][0].first) {
			std::cout << "read_crestlines: component ambiguous!" << std::endl;
			return -1;
		}
		if (m_Branch[ps+ev1][0].first-ls<0 || m_Branch[ps+ev1][0].first-ls>=num_lines) {
			std::cout << "read_crestlines: error component number!" << std::endl;
			return -1;
		}
		m_pCLInfo[m_Branch[ps+ev1][0].first].seg.push_back(CLI<FT>::CSeg(ps+ev1, ps+ev2, fi));
	}
	return 0;
}

void KevinVectorField::make_seg_consistent(void)
{
	for (int i=0; i<m_pCLInfo.size(); i++) {
		int num_seg = m_pCLInfo[i].seg.size();
		m_pCLInfo[i].dir.resize(num_seg);
		std::vector<short> scan(num_seg, 0);
		scan[num_seg/2] = 1;
		bool bFinished = false;
		while (!bFinished)
		{
			bFinished = true;
			for (int j=0; j<scan.size(); j++)
			{
				if (scan[j] != 1) {
					continue;
				}
				for (int k=0; k<num_seg; k++) {
					if ( k!=j && scan[k]==0)
					{
						if (m_pCLInfo[i].seg[j].v[0]==m_pCLInfo[i].seg[k].v[0]
							|| m_pCLInfo[i].seg[j].v[1]==m_pCLInfo[i].seg[k].v[1])
						{	// found a un-scanned neighbor
							m_pCLInfo[i].dir[k] *= -1.0;
							int tmp = m_pCLInfo[i].seg[k].v[0];
							m_pCLInfo[i].seg[k].v[0] = m_pCLInfo[i].seg[k].v[1];
							m_pCLInfo[i].seg[k].v[1] = tmp;
							scan[j] = 2;
							scan[k] = 1;
							bFinished = false;
						}
						else if (m_pCLInfo[i].seg[j].v[0]==m_pCLInfo[i].seg[k].v[1]
							|| m_pCLInfo[i].seg[j].v[1]==m_pCLInfo[i].seg[k].v[0])
						{	// found a un-scanned neighbor
							scan[j] = 2;
							scan[k] = 1;
							bFinished = false;
						}
					}
				}
			}
		}
	}
	m_bSegConsist = true;
}

void KevinVectorField::find_branch(Branch &branch)
{
	branch.clear();
	branch.resize(m_Points.size());
	for (int i=0; i<m_pCLInfo.size(); i++) {
		int num_seg = m_pCLInfo[i].seg.size();
		for (int j=0; j<num_seg; j++)
		{
			branch[m_pCLInfo[i].seg[j].v[0]].push_back(std::make_pair(j,0));
			branch[m_pCLInfo[i].seg[j].v[1]].push_back(std::make_pair(j,0));
		}
	}
}

void KevinVectorField::trace_seg_branch(CLI<FT> &NewCL, const CLI<FT> &CL, Branch &branch, int p, int s)
{
	NewCL.type = CL.type;
	NewCL.r = CL.r;
	NewCL.s = CL.s;
	NewCL.c = CL.c;
	int cp=p, cs, vi;
	if (cp == CL.seg[s].v[0]) {
		NewCL.seg.push_back(CL.seg[s]);
		cp = CL.seg[s].v[1];
		cs = s;
		vi = 1;
	} else if (cp == CL.seg[s].v[1]) {
		NewCL.seg.push_back(CL.seg[s]);
		cp = CL.seg[s].v[0];
		cs = s;
		vi = 0;
	} else {
		std::cout << "trace_seg_branch: branch info. error!" << std::endl;
	}
	bool bFound = true;
	while (branch[cp].size()==2 && bFound)
	{
		bFound = false;
		//
		branch[cp][0].second = -1;	// stepped on this point
		for (int i=0; i<CL.seg.size(); i++)
		{
			if (i == cs) {
				continue;
			}
			if (CL.seg[i].v[1-vi] == cp) {	// come sequentially
				NewCL.seg.push_back(CL.seg[i]);
				cp = CL.seg[i].v[vi];
				cs = i;
				bFound = true;
				break;
			}
			if (CL.seg[i].v[vi] == cp) {
				NewCL.seg.push_back(CLI<FT>::CSeg(CL.seg[i].v[1],CL.seg[i].v[0],CL.seg[i].fi));
				cp = CL.seg[i].v[1-vi];;
				cs = i;
				bFound = true;
				break;
			}
		}
	}
	if (!bFound) {
		std::cout << "trace_seg_branch: branch info. error!" << std::endl;
	}
	if (branch[cp].size() > 2) {
		for (int i=0; i<branch[cp].size(); i++) {
			if (branch[cp][i].first == cs) {
				branch[cp][i].second = 1;
				break;
			}
		}
	} else if (branch[cp].size() == 1) {	// must be branch[cp].size()==1
		branch[cp][0].second = -1;	// stepped on this point
	}
}

void KevinVectorField::del_crestlines(std::set<int> &dels, Branch &branch)
{
	// delete really
	std::set<int>::iterator it, jt;
	std::vector<int> rslt_line_no(m_pCLInfo.size());
	int mv_id=0, shift=0;
	for (it=dels.begin(); it!=dels.end(); it++) {
		for (int i=mv_id; i<(*it); i++) {
			rslt_line_no[i] = i-shift;
		}
		mv_id = (*it)+1;
		shift++;
	}
	for (int i=mv_id; i<m_pCLInfo.size(); i++) {
		rslt_line_no[i] = i-shift;
	}
	for (it=dels.begin(); it!=dels.end(); it++) {
		m_pCLInfo.erase(m_pCLInfo.begin()+(*it));
		for (jt=dels.begin(); jt!=dels.end(); jt++) {
			if (*jt > *it) {
				//(*jt)--;
        jt--;
			}
		}
	}
	// update line# in branch nodes
	for (int i=0; i<m_Points.size(); i++)
	{
		for (int j=0; j<branch[i].size(); j++) {
			branch[i][j].first = rslt_line_no[branch[i][j].first];
		}
	}
}

void KevinVectorField::make_seg_sequent(void)
{
	std::list<CLI<FT>::CSeg> seg_order;
	std::vector<short> seg_added;
	for (int i=0; i<m_pCLInfo.size(); i++) {
		int num_seg = m_pCLInfo[i].seg.size();
		seg_added.clear();
		seg_added.resize(num_seg, 0);
		seg_order.clear();
		seg_order.push_back(m_pCLInfo[i].seg[0]);
		seg_added[0] = 1;
		int vl = m_pCLInfo[i].seg[0].v[0];
		int vr = m_pCLInfo[i].seg[0].v[1];
		int t = 0;
		while (seg_order.size()<num_seg && t<num_seg) {	// find t times
			for (int j=0; j<num_seg; j++) {
				if (seg_added[j] == 1) {
					continue;
				}
				if (m_pCLInfo[i].seg[j].v[0] == vr) {
					seg_order.push_back(m_pCLInfo[i].seg[j]);
					seg_added[j] = 1;
					vr = m_pCLInfo[i].seg[j].v[1];
				} else if (m_pCLInfo[i].seg[j].v[1] == vl) {
					seg_order.push_front(m_pCLInfo[i].seg[j]);
					seg_added[j] = 1;
					vl = m_pCLInfo[i].seg[j].v[0];
				}
			}
			t++;
		}
		if (t == num_seg) {
			std::cout << "make_seg_sequent: max times of loop exceeded for line " << i << " !" << std::endl;
			continue;
		}
		int j;
		std::list<CLI<FT>::CSeg>::iterator it;
		for (it=seg_order.begin(), j=0; it!=seg_order.end(); it++, j++) {
				m_pCLInfo[i].seg[j] = *it;
		}
	}
	m_bSegSequent = true;
}

void KevinVectorField::compute_smooth_dir(void)
{
	if (!m_bSegConsist) {
		make_seg_consistent();
	}
	if (!m_bSegSequent) {
		make_seg_sequent();
	}
	for (int i=0; i<m_pCLInfo.size(); i++)
	{
		int num_seg = m_pCLInfo[i].seg.size();
		int step = __max(__min(num_seg*0.03, 5), 1);
		m_pCLInfo[i].dir.resize(num_seg);
		for (int j=0; j<num_seg; j++) {
			int ts = __min(num_seg-1, j+step);
			int hs = __max(ts-step,0);
			m_pCLInfo[i].dir[j] = m_Points[m_pCLInfo[i].seg[ts].v[1]] - m_Points[m_pCLInfo[i].seg[hs].v[0]];
			m_pCLInfo[i].dir[j].normalize();
		}
		// perform a smoothing
		for (int j=0; j<num_seg; j++) {
			Vector3 v;
			if ((j-3)>=0) {
				v += m_pCLInfo[i].dir[j-3];
			}
			if ((j-2)>=0) {
				v += m_pCLInfo[i].dir[j-2] * 2.0;
			}
			if ((j-1)>=0) {
				v += m_pCLInfo[i].dir[j-1] * 4.0;
			}
			v += m_pCLInfo[i].dir[j] * 4.0;
			if ((j+1)<num_seg) {
				v += m_pCLInfo[i].dir[j+1] * 2.0;
			}
			if ((j+2)<num_seg) {
				v += m_pCLInfo[i].dir[j+2];
			}
			v.normalize();
			m_pCLInfo[i].dir[j] = v;
		}
	}
}

int KevinVectorField::merge_two_lines(CLI<FT> &NewCL, const CLI<FT> &CL1, bool bL1R, const CLI<FT> &CL2, bool bL2R, bool bCP)
{
	if (CL1.type != CL2.type) {
		std::cout << "merge_two_lines: trying to merge two lines with different types!" << std::endl;
		return -1;
	}
	FT lambda = (FT)CL1.seg.size() / (FT)(CL1.seg.size()+CL2.seg.size());
	NewCL.r = CL1.r * lambda + CL2.r * (1.0-lambda);
	NewCL.s = CL1.s * lambda + CL2.s * (1.0-lambda);
	NewCL.c = CL1.c * lambda + CL2.c * (1.0-lambda);
	NewCL.type = CL1.type;
	NewCL.tag = (CL1.tag!=0||CL2.tag!=0) ? 1 : 0;
	NewCL.seg.resize(CL1.seg.size()+CL2.seg.size());
	NewCL.dir.resize(CL1.dir.size()+CL2.dir.size());
	//////////////////////////////////////////////////////////////////////////
	if (bL1R) {
		int n = CL1.seg.size();
		for (int i=0; i<n; i++) {
			NewCL.seg[i] & CL1.seg[n-1-i];
		}
		n = CL1.dir.size();
		for (int i=0; i<n; i++) {
			NewCL.dir[i] & CL1.dir[n-1-i];
		}
	} else {
		std::copy(CL1.seg.begin(), CL1.seg.end(), NewCL.seg.begin());
		std::copy(CL1.dir.begin(), CL1.dir.end(), NewCL.dir.begin());
	}
	//////////////////////////////////////////////////////////////////////////
	if (!bCP) {	// have common point
		NewCL.seg.resize(NewCL.seg.size()+1);
		int fi1 = (bL1R) ? CL1.seg.front().fi : CL1.seg.back().fi;
		int fi2 = (bL2R) ? CL2.seg.back().fi : CL2.seg.front().fi;
		int vi1 = (bL1R) ? CL1.seg.front().v[0] : CL1.seg.back().v[1];
		int vi2 = (bL2R) ? CL2.seg.back().v[1] : CL2.seg.front().v[0];
		if (fi1 == fi2) {
			NewCL.seg[CL1.seg.size()].set(vi1, vi2, fi1);
		} else {
			NewCL.seg[CL1.seg.size()].set(vi1, vi2, -1);
		}
		if (CL1.dir.size()!=0 && CL2.dir.size()!=0) {
			NewCL.dir.resize(NewCL.dir.size()+1);
			const Vector3 &dir1 = (bL1R) ? CL1.dir.front() : CL1.dir.back();
			const Vector3 &dir2 = (bL2R) ? CL2.dir.back() : CL2.dir.front();
			NewCL.dir[CL1.dir.size()] = dir1 + dir2;
			NewCL.dir[CL1.dir.size()].normalize();
		}
	}
	//////////////////////////////////////////////////////////////////////////
	int ss = (!bCP) ? CL1.seg.size()+1 : CL1.seg.size();
	int ds = (!bCP && CL1.dir.size()!=0 && CL2.dir.size()!=0) ? CL1.dir.size()+1 : CL1.dir.size();
	if (bL2R) {
		int n = CL2.seg.size();
		for (int i=0; i<n; i++) {
			NewCL.seg[ss+i] & CL2.seg[n-1-i];
		}
		n = CL2.dir.size();
		for (int i=0; i<n; i++) {
			NewCL.dir[ds+i] & CL2.dir[n-1-i];
		}
	} else {
		std::copy(CL2.seg.begin(), CL2.seg.end(), NewCL.seg.begin()+ss);
		std::copy(CL2.dir.begin(), CL2.dir.end(), NewCL.dir.begin()+ds);
	}
	return 0;
}

void KevinVectorField::weld_at_branches(Branch &branch)
{
	std::set<int> dels;
	for (int i=0; i<branch.size(); i++)
	{
		if (branch[i].size() < 3) {
			continue;
		}
		FT avg_len = 0;
		for (int j=0; j<branch[i].size(); j++) {
			avg_len += m_pCLInfo[branch[i][j].first].seg.size();
		}
		avg_len /= (FT)branch[i].size();
		std::vector<int> Left;
		for (int j=0; j<branch[i].size(); j++) {
			if (m_pCLInfo[branch[i][j].first].seg.size() > avg_len*0.12) {
				Left.push_back(j);
			}
		}
		if (Left.size() == 2) {
			int ls = m_pCLInfo.size();
			m_pCLInfo.resize(m_pCLInfo.size()+1);
			const CLI<FT> &CL1 = m_pCLInfo[branch[i][Left[0]].first];
			const CLI<FT> &CL2 = m_pCLInfo[branch[i][Left[1]].first];
			merge_two_lines(m_pCLInfo[ls], CL1, (CL1.seg.back().v[1]!=i), CL2, (CL2.seg.front().v[0]!=i), true);
			// effect to other branches
			for (int j=0; j<branch.size(); j++) {
				if (j==i) {
					continue;
				}
				for (int k=0; k<branch[j].size(); k++) {
					if (branch[j][k].first==branch[i][Left[0]].first
						|| branch[j][k].first==branch[i][Left[1]].first) {
						branch[j][k].first = ls;
					}
				}
			}
			// record to delete old branches at this branch node
			for (int j=0; j<branch[i].size(); j++) {
				dels.insert(branch[i][j].first);
			}
			branch[i].resize(1);
			branch[i][0].first = ls;	// update line#: only one line pass through this branch now
		}
	}
	// delete really
	del_crestlines(dels, branch);
}

void KevinVectorField::clean_lines(void)
{
  if (!m_bSegConsist) {
		make_seg_consistent();
	}
	// first, clean lines with branches and order the points in the line order
	Branch pSegBranch;
	// find segment branches at each point
	find_branch(pSegBranch);
	// trace segment branches to get line branches
	std::set<int> dels;
	for (int i=0; i<m_Points.size(); i++)
	{
		if (pSegBranch[i].size() < 3) {
			continue;
		}
		int iOriginBN = m_Branch[i][0].first;	// record original
		m_Branch[i].clear();	// clear this big line and seperate it now
		for (int j=0; j<pSegBranch[i].size(); j++)
		{
			if (pSegBranch[i][j].second == 1) {
				// this exit has been accessed, delete it from this branch node
				pSegBranch[i][j].second = -1;	// mark as deleted
				continue;
			}
			int ls = m_pCLInfo.size();
			m_pCLInfo.resize(m_pCLInfo.size()+1);
			trace_seg_branch(m_pCLInfo[ls], m_pCLInfo[iOriginBN], pSegBranch, i, pSegBranch[i][j].first);
			m_Branch[i].push_back(std::make_pair(ls,0));
			//////////////////////////////////////////////////////////////////////////
			for (int k=0; k<pSegBranch.size(); k++) {
				if (pSegBranch[k].size()>0 && pSegBranch[k].size()<3 && pSegBranch[k][0].second==-1) {
					m_Branch[k][0].first = ls;
					pSegBranch[k][0].second = 0;
				}
			}
		}
		dels.insert(iOriginBN);
	}
	// delete really
	del_crestlines(dels, m_Branch);
	//////////////////////////////////////////////////////////////////////////
	// NOTE: from now on, m_PInL should not be used any more because i did no
	//		 update for it. Actually, similar information can be found in branch.
	//		 What's more, a point might belong to multiple lines currently.
	//////////////////////////////////////////////////////////////////////////
	// make sequence for the the rest lines
	make_seg_sequent();
	// compute smooth direction
	compute_smooth_dir();
	// weld lines
	weld_at_branches(m_Branch);
	//weld_close_lines(branch);
}

void KevinVectorField::compute_line_length()
{
	for (int i=0; i<m_pCLInfo.size(); i++) {
		FT dLen = 0.0;
		for (int j=0; j<m_pCLInfo[i].seg.size(); j++) {
			m_pCLInfo[i].seg[j].l = m_Points[m_pCLInfo[i].seg[j].v[0]].distance(m_Points[m_pCLInfo[i].seg[j].v[1]]);
			dLen += m_pCLInfo[i].seg[j].l;
		}
		m_pCLInfo[i].l = dLen;
	}
	m_bCompLen = true;
}

void KevinVectorField::get_crest_range(void)
{
	if (!m_bCompLen) {
		compute_line_length();
	}
	m_dMinR = FT_MAX;	// minimum ridgeness
	m_dMaxR = 0.0;		// maximum ridgeness
	m_dMinS = FT_MAX;	// minimum sphericalness
	m_dMaxS = 0.0;		// maximum sphericalness
	m_dMinC = FT_MAX;	// minimum cyclideness
	m_dMaxC = 0.0;		// maximum cyclideness
	m_dMinL = FT_MAX;	// minimum length
	m_dMaxL = 0.0;		// maximum length
	for (int i=0; i<m_pCLInfo.size(); i++) {
		if (m_pCLInfo[i].r > m_dMaxR) {
			m_dMaxR = m_pCLInfo[i].r;
		}
		if (m_pCLInfo[i].r < m_dMinR) {
			m_dMinR = m_pCLInfo[i].r;
		}
		if (m_pCLInfo[i].s > m_dMaxS) {
			m_dMaxS = m_pCLInfo[i].s;
		}
		if (m_pCLInfo[i].s < m_dMinS) {
			m_dMinS = m_pCLInfo[i].s;
		}
		if (m_pCLInfo[i].c > m_dMaxC) {
			m_dMaxC = m_pCLInfo[i].c;
		}
		if (m_pCLInfo[i].c < m_dMinC) {
			m_dMinC = m_pCLInfo[i].c;
		}
		if (m_pCLInfo[i].l > m_dMaxL) {
			m_dMaxL = m_pCLInfo[i].l;
		}
		if (m_pCLInfo[i].l < m_dMinL) {
			m_dMinL = m_pCLInfo[i].l;
		}
	}
}

void KevinVectorField::exportCCL()
{
  VertexList vertex = shape_model->getShapeVertexList();
  std::vector<STLVectori> allCurves = shape_model->getShapeCrestLine();
  FILE *fp;
  fp = fopen((shape_model->getDataPath() + "/" + shape_model->getFileName() + ".ccl").c_str(),"w+");
  int line_size = allCurves.size();
  int point_size = 0;
  for(int i = 0; i < allCurves.size(); i ++)
  {
    point_size += allCurves[i].size();
  }
  int seg_size = point_size - line_size;
  fprintf(fp, "%d\n", point_size);
  fprintf(fp, "%d\n", seg_size);
  fprintf(fp, "%d\n", line_size);
  for(int i = 0; i < allCurves.size(); i ++)
  {
    for(int j = 0; j < allCurves[i].size(); j ++)
    {
      fprintf(fp, "%f %f %f %d\n", vertex[allCurves[i][j] * 3], vertex[allCurves[i][j] * 3 + 1], vertex[allCurves[i][j] * 3 + 2], i);
    }
  }
  for(int i = 0; i < line_size; i ++)
  {
    fprintf(fp, "%f %f %f %d\n", 0.0, 0.0, 0.0, 1);
  }
  std::vector<std::vector<Vector3>> dir;
  dir.resize(allCurves.size());
  for(int i = 0; i < allCurves.size(); i ++)
  {
    dir[i].resize(allCurves[i].size() - 1);
  }
  for(int i = 0; i < allCurves.size(); i ++)
  {
    int num_seg = allCurves[i].size() - 1;
    int step = __max(__min(num_seg*0.03, 5), 1) + 1;
    for(int j = 0; j < num_seg; j ++)
    {
      int ts = __min(num_seg-1, j+step);
			int hs = __max(ts-step,0);
      dir[i][j].set(vertex[allCurves[i][ts] * 3] - vertex[allCurves[i][hs] * 3], vertex[allCurves[i][ts] * 3 + 1] - vertex[allCurves[i][hs] * 3 + 1], vertex[allCurves[i][ts] * 3 + 2] - vertex[allCurves[i][hs] * 3 + 2]);
      dir[i][j].normalize();
    }
    for (int j=0; j<num_seg; j++) {
			Vector3 v;
			if ((j-3)>=0) {
				v += dir[i][j-3];
			}
			if ((j-2)>=0) {
				v += dir[i][j-2] * 2.0;
			}
			if ((j-1)>=0) {
				v += dir[i][j-1] * 4.0;
			}
			v += dir[i][j] * 4.0;
			if ((j+1)<num_seg) {
				v += dir[i][j+1] * 2.0;
			}
			if ((j+2)<num_seg) {
				v += dir[i][j+2];
			}
			v.normalize();
			dir[i][j] = v;
		}
  }
  std::vector<int> face_id;
  PolygonMesh* mesh = shape_model->getPolygonMesh();
  for(int i = 0; i < allCurves.size(); i ++)
  {
    for(int j = 0; j < allCurves[i].size() - 1; j ++)
    {
      int m = allCurves[i][j], n = allCurves[i][j + 1];
      PolygonMesh::Vertex v(m);
      bool isConnected = FALSE;
      for (auto hvc_it : mesh->halfedges(v))
      {
        if (mesh->to_vertex(hvc_it).idx() == n)
        {
          //poly_mesh->opposite_halfedge(hvc_it)
          face_id.push_back(mesh->face(hvc_it).idx());
          isConnected = TRUE;
        }
      }
      if(isConnected == FALSE)
      {
        face_id.push_back(-1);
      }
    }
  }
  int count1 = 0, count2 = 1, count3 = 0;
  for(int i = 0; i < allCurves.size(); i ++)
  {
    for(int j = 0; j < allCurves[i].size() - 1; j ++)
    {
      fprintf(fp, "%d %d %d %f %f %f\n", count1, count2, face_id[count3], dir[i][j].x, dir[i][j].y, dir[i][j].z);
      if(j == allCurves[i].size() - 2)
      {
        count1 += 2;
        count2 += 2;
      }
      else
      {
        count1 ++;
        count2 ++;
      }
      count3 ++;
    }
  }
  fclose(fp);
}

int KevinVectorField::readCCL(std::ifstream &stream)
{
	std::string buf;
	int num_points, num_segs, num_lines;
	stream >> num_points;
  if (num_points<0 || num_points>=(shape_model->getShapeVertexList().size() / 3)) {
		return -1;
	}
	stream >> num_segs;
	if (num_segs < 0) {
		return -1;
	}
	stream >> num_lines;
	if (num_lines < 0) {
		return -1;
	}
	if (num_lines == 0) {
		return 0;
	}
	int ls = m_pCLInfo.size();
	int ps = m_Points.size();
	m_pCLInfo.resize(m_pCLInfo.size() + num_lines);
	m_Points.resize(m_Points.size() + num_points);
	m_Branch.resize(m_Branch.size() + num_points);
	for (int i=0; i<num_points; i++)
	{
		m_Branch[ps+i].resize(1);
		stream >> m_Points[ps+i].x >> m_Points[ps+i].y >> m_Points[ps+i].z >> m_Branch[ps+i][0].first;
		m_Branch[ps+i][0].first += ls;
		m_Branch[ps+i][0].second = 0;
	}
	for (int i=0; i<num_lines; i++)
	{
		int type;
		stream >> m_pCLInfo[ls+i].r >> m_pCLInfo[ls+i].s >> m_pCLInfo[ls+i].c >> type;
		m_pCLInfo[ls+i].type = (Crest_type)type;
	}
	int ev1, ev2, fi;
	FT  x, y, z;
	for (int i=0; i<num_segs; i++)
	{
		stream >> ev1 >> ev2 >> fi;
		if (m_Branch[ps+ev1][0].first != m_Branch[ps+ev2][0].first) {
			std::cout << "read_crestlines: component ambiguous!" << std::endl;
			return -1;
		}
		if (m_Branch[ps+ev1][0].first-ls<0 || m_Branch[ps+ev1][0].first-ls>=num_lines) {
			std::cout << "read_crestlines: error component number!" << std::endl;
			return -1;
		}
		m_pCLInfo[m_Branch[ps+ev1][0].first].seg.push_back(CLI<FT>::CSeg(ps+ev1, ps+ev2, fi));
		stream >> x >> y >> z;
		m_pCLInfo[m_Branch[ps+ev1][0].first].dir.push_back(Vector3(x, y, z));
	}
	return 0;
}

void KevinVectorField::load_crestlines()
{
  std::string sDirName = shape_model->getDataPath();
  std::string name = sDirName + "/" + shape_model->getFileName() + ".ccl";
  std::cout << name << std::endl;
  FILE *fp;
  if(fp = fopen(name.c_str(), "r"))
  {
    fclose(fp);
  }
  else
  {
    exportCCL();
  }
 // std::string name1 = sDirName + "/ridges.txt", name2 = sDirName + "/ravines.txt";
 // std::ifstream stream1(name1);
 // std::ifstream stream2(name2);
 // m_pCLInfo.clear();
 // if (read_crestlines(stream1, CONCAVE) != 0) 
 // {
	//	std::cout << "Load concave crest lines error!" << std::endl;
	//	return ;
	//}
 // if (read_crestlines(stream2, CONVEX) != 0) 
 // {
	//	std::cout << "Load convex crest lines error!" << std::endl;
	//	return ;
	//}
	//m_bSegConsist = false;
	//m_bSegSequent = false;
	//m_bCompLen = false;
	//// clean the crest lines
	//clean_lines();

  
  std::ifstream stream(name);
  if (readCCL(stream) != 0) {
		std::cout << "Load convex crest lines error!" << std::endl;
		return ;
	}
  m_bSegConsist = true;
	m_bSegSequent = true;
	m_bCompLen = false;

	compute_line_length();
	// get min-max of stength and sharpness
	get_crest_range();
}

FT KevinVectorField::sqdPnt2LineSeg(const Vector3 &ls, const Vector3 &le, const Vector3 &p)
{	// Square distance from a point to a line segment
	FT len = ls.sqauredistance(le);
	FT t = (p-ls).dot(le-ls) / len;
	if (t < 0.0) {
		return p.sqauredistance(ls);
	}
	if (t > 1.0) {
		return p.sqauredistance(le);
	}
	Vector3 pp = ls + (le-ls) * t;
	return p.sqauredistance(pp);
}

void KevinVectorField::compute_site_vectors(void)
{
	/*if (m_pMesh == NULL)
	{ MSG_BOX_ERROR("No mesh information!"); return; }
*/
	MAP_IVS::iterator it;
	int vid;
	//FT w;
	for (int i=0; i<m_pCLInfo.size(); i++)
	{
		if (m_pCLInfo[i].tag == 0) {
			continue;
		}
		for (int j=0; j<m_pCLInfo[i].seg.size(); j++) {
			if (j%5!=0 || m_pCLInfo[i].seg[j].fi==-1) {
				continue;
			}
			// find the closest vertex in triangle m_pCLInfo[i].seg[j].fi
			int fi = m_pCLInfo[i].seg[j].fi;
      
			Vector3 v;
			const Vector3 &ev1 = m_Points[m_pCLInfo[i].seg[j].v[0]];
			const Vector3 &ev2 = m_Points[m_pCLInfo[i].seg[j].v[1]];
			FT min_sqd = std::numeric_limits<FT>::max();
			int min_k = 0;
			for (int k=0; k<3; k++) {
				//const Point_3 &vp = m_pMesh->fvp(fi, k);
        int vertex_id;
        vertex_id = shape_model->getShapeFaceList()[3 * fi + k];
        Vector3f vp;
        vp << shape_model->getShapeVertexList()[vertex_id * 3], shape_model->getShapeVertexList()[vertex_id * 3 + 1], shape_model->getShapeVertexList()[vertex_id * 3 + 2];
				v.set(vp.x(), vp.y(), vp.z());
				FT sqd = sqdPnt2LineSeg(ev1, ev2, v);
				if (sqd < min_sqd) {
					min_sqd = sqd;
					min_k = k;
				}
			}
			//int vid = m_pMesh->fvid(fi, min_k);
      int vid = shape_model->getShapeFaceList()[3 * fi + min_k];
			//w = 1.0 / min_sqd;
			// check if existed
			if ((it=m_hvf_sites.find(vid)) != m_hvf_sites.end()) {
				if ((it->second).second == 0) {	// existed and first time handling in this round
					(it->second).first.set(0.0, 0.0, 0.0);	// reset
					(it->second).second = 2;	// up-to-date, handled this round
				}
				(it->second).first += m_pCLInfo[i].dir[j];
			} else {
				m_hvf_sites.insert(std::make_pair(vid, std::make_pair(m_pCLInfo[i].dir[j],1)));
			}
		}
	}
	// normalize all site vectors
	std::vector<MAP_IVS::iterator> deli;
	//m_pMesh->clear_hvf_sites();
	for (it=m_hvf_sites.begin(); it!=m_hvf_sites.end(); it++)
	{
		if ((it->second).second == 0) {	// not appear this round, should be deleted
      m_pMesh_hvf_site.insert(std::make_pair(it->first, std::make_pair(Vector3(0.0,0.0,0.0), -1)));
			deli.push_back(it);
		} else {
			if ((it->second).second == 2) {	// 2 == 0 : distinctive from old, to-be-deleted 0
				(it->second).second = 0;
			}
			Vector3 v((it->second).first);
			v.normalize();
			v *= 0.15;
			m_pMesh_hvf_site.insert(std::make_pair(it->first, std::make_pair(v, (it->second).second)));
			// clear to be up-to-date
			(it->second).second = 0;
		}
	}
	// delete those should be deleted at once in this->m_hvf_sites
	for (unsigned int i=0; i<deli.size(); i++) {
		m_hvf_sites.erase(deli[i]);
	}
}

void KevinVectorField::compute_s_hvf(void)
{
  load_crestlines();
  compute_site_vectors();
	if (m_pMesh_hvf_site.empty()) {
		return;
	}
	cholmod_common c;
	cholmod_start(&c);	// start CHOLMOD
	if (m_s_hvf_F == NULL) {	// if not existed, compute new factorization
    CMatrix *AM_hf = new CMatrix((int)(shape_model->getShapeVertexList().size() / 3), true, &c);
		set_s_hvf_lap_uniform_PNT(AM_hf);
		m_s_hvf_F = cholmod_analyze((cholmod_sparse*)AM_hf->get_cholmod_sparse(), &c);	// analyze
		cholmod_factorize((cholmod_sparse*)AM_hf->get_cholmod_sparse(), m_s_hvf_F, &c);	// factorize
		delete AM_hf;
	} else {
		CMatrix *cmC = new CMatrix((int)(shape_model->getShapeVertexList().size() / 3), false, &c);
		cholmod_sparse *Cnew;
		bool bNeedDown = false;
		bool bNeedUp = false;
		// build downdate sparse
		std::vector<MAP_IVS::iterator> its;
		for (MAP_IVS::iterator it=m_pMesh_hvf_site.begin(); it!=m_pMesh_hvf_site.end(); it++) {
			if ((it->second).second == -1) {	// -1: to be downdated
				cmC->set_coef(it->first, it->first, SQBN);
				its.push_back(it);
				bNeedDown = true;
			}
		}
		// delete really
		for (unsigned int i=0; i<its.size(); i++) {
			m_pMesh_hvf_site.erase(its[i]);
		}
		if (bNeedDown) {	// if needed, perform downdating
			Cnew = cholmod_submatrix((cholmod_sparse*)cmC->get_cholmod_sparse(), (int*)m_s_hvf_F->Perm,
										m_s_hvf_F->n, NULL, -1, TRUE, TRUE, &c);
			cholmod_updown(FALSE, Cnew, m_s_hvf_F, &c);
			cholmod_free_sparse(&Cnew, &c);
			cmC->clear_sparse();
		}
		// build update sparse
		for (MAP_IVS::iterator it=m_pMesh_hvf_site.begin(); it!=m_pMesh_hvf_site.end(); it++) {
			if ((it->second).second == 1) {	// +1: to be updated
				cmC->set_coef(it->first, it->first, SQBN);
				(it->second).second = 0;	// turn it back to 0: up-to-date
				bNeedUp = true;
			}
		}
		if (bNeedUp) {	// if needed, perform updating
			Cnew = cholmod_submatrix((cholmod_sparse*)cmC->get_cholmod_sparse(), (int*)m_s_hvf_F->Perm,
										m_s_hvf_F->n, NULL, -1, TRUE, TRUE, &c);
			cholmod_updown(TRUE, Cnew, m_s_hvf_F, &c);
			cholmod_free_sparse(&Cnew, &c);
		}
		delete cmC;
	}
	CVector *RHV_hf = new CVector((int)(shape_model->getShapeVertexList().size() / 3), &c);
	if (m_s_vf == NULL) m_s_vf = new Vector3 [(shape_model->getShapeVertexList().size() / 3)];
	for (int d=0; d<3; d++)
	{
		set_s_hvf_rhs_vec(RHV_hf, d);	// set rhs vector
		cholmod_dense *X = cholmod_solve(CHOLMOD_A, m_s_hvf_F, (cholmod_dense*)RHV_hf->get_cholmod_dense(), &c);	// solve Ax=b
		for (int i=0; i<(shape_model->getShapeVertexList().size() / 3); i++) m_s_vf[i][d]=(FT)((double*)X->x)[i];
		cholmod_free_dense(&X, &c);
	}
	delete RHV_hf;
	// projection
	project_harmonic_vector_field(m_s_vf, 0.0, 1.0);
	cholmod_finish(&c);		// end CHOLMOD
	m_is_s_hvf_utd = true;

  PolygonMesh* src_poly_mesh = shape_model->getPolygonMesh();
  PolygonMesh::Vertex_attribute<Vec3> tangent = src_poly_mesh->vertex_attribute<Vec3>("v:tangent");
  for (auto vit : src_poly_mesh->vertices())
  {
    tangent[vit] = Vec3(m_s_vf[vit.idx()].x, m_s_vf[vit.idx()].y, m_s_vf[vit.idx()].z);
  }

  float vis_scale = shape_model->getBoundBox()->getRadius() / 20;
  for(int i = 0; i < shape_model->getShapeVertexList().size() / 3; i ++)
  {
    Vector3f start;
    start << (shape_model->getShapeVertexList())[3 * i], (shape_model->getShapeVertexList())[3 * i + 1], (shape_model->getShapeVertexList())[3 * i + 2];
    Vector3f end;
    //end = start + vector_field[i];
    end << start.x() + vis_scale * m_s_vf[i].x, start.y() + vis_scale * m_s_vf[i].y, start.z() + vis_scale * m_s_vf[i].z;
    actors[0].addElement(start.x(), start.y(), start.z(), 1.0, 0.0, 0.0);
    actors[2].addElement(start.x(), start.y(), start.z(), 0.0, 0.0, 0.0);
    actors[2].addElement(end.x(), end.y(), end.z(), 0.0, 0.0, 0.0);
  }
  std::cout << "Computing vector field is finished ! " << std::endl;
}

void KevinVectorField::set_s_hvf_lap_uniform_PNT(CMatrix *tmAssLM)
{
	FT diag_sum(0.0);
	short *tag = new short [(shape_model->getShapeVertexList().size() / 3)];
	memset(tag, 0, (shape_model->getShapeVertexList().size() / 3)*sizeof(short));
	for (MAP_IVS::iterator it=m_pMesh_hvf_site.begin(); it!=m_pMesh_hvf_site.end(); it++) {
		tag[it->first] = 1;
		(it->second).second = 0;
	}
	// Set Laplacian matrix
	// BNM: d+n is equivallent to d*(1+n/d), where the big number is (1+n/d)
	PolygonMesh* mesh = shape_model->getPolygonMesh();
  PolygonMesh::Vertex_iterator vit = mesh->vertices_begin(), vend = mesh->vertices_end();
  for(; vit != vend; ++ vit)
  {
    diag_sum = 0.0;
    int vi = (*vit).idx();
    PolygonMesh::Halfedge_around_vertex_circulator pHalfEdge, end;
    pHalfEdge = end = mesh->halfedges(*vit);
    do
    {
      int vj = mesh->to_vertex(*pHalfEdge).idx();
      double wij = 1.0;
      diag_sum += wij;
      tmAssLM->set_coef(vi, vj, -wij);
    }while(++pHalfEdge != end);
    if (tag[vi] == 1) {
			tmAssLM->set_coef(vi, vi, diag_sum+BN);
		} else {
			tmAssLM->set_coef(vi, vi, diag_sum);
		}
  }
  /*for (int i=0; i<(shape_model->getShapeVertexList().size() / 3); i++)
	{
		pHalfEdge = index_to_vertex_map[i]->vertex_begin();
		end = pHalfEdge;
		diag_sum = 0.0;
		CGAL_For_all(pHalfEdge, end)
		{
			tmAssLM->set_coef(i, pHalfEdge->opposite()->vertex()->id(), -1.0);
			diag_sum += 1.0;
		}
		if (tag[i] == 1) {
			tmAssLM->set_coef(i, i, diag_sum+BN);
		} else {
			tmAssLM->set_coef(i, i, diag_sum);
		}
	}*/
	//SAFE_DELETE_ARRAY(tag);
}

void KevinVectorField::set_s_hvf_rhs_vec(CVector *tvRHSV, int dim)
{
	// BNM: d+n is equivallent to d*(1+n/d), where the big number is (1+n/d)
	tvRHSV->clear_zero();
	MAP_IVS::iterator it;
	for (it=m_pMesh_hvf_site.begin(); it!=m_pMesh_hvf_site.end(); it++) {
		(*tvRHSV)[it->first] = BN * ((it->second).first)[dim];
	}
}

void KevinVectorField::project_harmonic_vector_field(Vector3 *hvf, FT low=1.0, FT high=1.0)
{
	if (hvf==NULL || low>high || low<0.0) {
		return;
	}
	Vector3 vn, pvec;
	if (low == high) {
		m_s_hvm_min = std::numeric_limits<FT>::max();
		m_s_hvm_max = std::numeric_limits<FT>::min();
		for (int i=0; i<(shape_model->getShapeVertexList().size() / 3); i++) {
      vn.set((shape_model->getShapeNormalList())[3 * i], (shape_model->getShapeNormalList())[3 * i + 1], (shape_model->getShapeNormalList())[3 * i + 2]);
			pvec = hvf[i].cross(vn);
			FT m(0.0);
			if (IsZeroL(pvec[0])&&IsZeroL(pvec[1])&&IsZeroL(pvec[2])) {
				hvf[i].set(0.0, 0.0, 0.0);
			} else {
				hvf[i] = vn.cross(pvec);
				m = hvf[i].magnitude();
			}
			if (m > m_s_hvm_max) {
				m_s_hvm_max = m;
			}
			if (m < m_s_hvm_min) {
				m_s_hvm_min = m;
			}
		}
	} else {
		FT mmax=std::numeric_limits<FT>::min();
		FT mmin=std::numeric_limits<FT>::max();
		std::vector<FT> mag((shape_model->getShapeVertexList().size() / 3), 0.0);
		for (int i=0; i<(shape_model->getShapeVertexList().size() / 3); i++) {
			vn.set((shape_model->getShapeNormalList())[3 * i], (shape_model->getShapeNormalList())[3 * i + 1], (shape_model->getShapeNormalList())[3 * i + 2]);
			pvec = hvf[i].cross(vn);
			if (IsZeroL(pvec[0])&&IsZeroL(pvec[1])&&IsZeroL(pvec[2])) {
				hvf[i].set(0.0, 0.0, 0.0);
				mag[i] = 0.0;
			} else {
				hvf[i] = vn.cross(pvec);
				mag[i] = hvf[i].magnitude();
			}
			if (mag[i] > mmax) {
				mmax = mag[i];
			}
			if (mag[i] < mmin) {
				mmin = mag[i];
			}
		}
		FT scl = (high - low) / (mmax - mmin);
		for (int i=0; i<(shape_model->getShapeVertexList().size() / 3); i++) {
			hvf[i] *= ((low + scl*(mag[i]-mmin)) / mag[i]);
		}
		m_s_hvm_min = low;
		m_s_hvm_max = high;
	}
}

void KevinVectorField::getDrawableActors(std::vector<GLActor>& actors)
{
  actors = this->actors;
}

