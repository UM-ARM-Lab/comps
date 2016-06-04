#ifndef  ELASTICSTRIPS_H
#define  ELASTICSTRIPS_H

#include <cdd/setoper.h>
#include <cdd/cdd.h>
// #include "ContactRegion.h"

class ElasticStrips : public ModuleBase
{
  public:
    ElasticStrips(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("RunElasticStrips",boost::bind(&ElasticStrips::RunElasticStrips,this,_1,_2),
                        "Run the Elastic Strips Planner");

        _esEnv = penv;

    }
    virtual ~ElasticStrips() {}

    int RunElasticStrips(ostream& sout, istream& sinput);

  private:

    void SetActiveRobots(const std::vector<RobotBasePtr >& robots);

    void InitPlan(boost::shared_ptr<ESParameters> params);
    OpenRAVE::PlannerStatus PlanPath(TrajectoryBasePtr ptraj);
    void LoadContactRegions(); // new elastic strips
    void DecideContactConsistentTransform(TrajectoryBasePtr ptraj);
    void FindNearestContactRegion();
    void FindContactRegions();
    void FindContactConsistentManipTranslation(TrajectoryBasePtr ptraj);
    Transform ForwardKinematics(std::vector<dReal> qs,string manip_name);
    dReal TransformDifference(dReal * dx,Transform tm_ref, Transform tm_targ);

    void GetRepulsiveVector(Vector& repulsive_vector, std::multimap<string,Vector>::iterator& control_point);
    void GetInternalVector(Vector& internal_vector, TrajectoryBasePtr ptraj, string control_link, size_t w);

    void UpdateZRPYandXYJacobianandStep(Transform taskframe_in, size_t w);
    void UpdateZRPYandXYJacobianandStep_old(Transform taskframe_in, size_t w);
    void UpdateCOGJacobianandStep(Transform taskframe_in, size_t w);
    void UpdateOAJacobianandStep(Transform taskframe_in, TrajectoryBasePtr ptraj, size_t w);
    void UpdatePCJacobianandStep(Transform taskframe_in, size_t w);

    void QuatToRPY(Transform tm, dReal& psi, dReal& theta, dReal& phi);
    int invConditioningBound(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed);
    void RemoveBadJointJacobianCols(NEWMAT::Matrix& J, size_t w);

    NEWMAT::ColumnVector CalculateStep();
    void PrintMatrix(dReal* pMatrix, int numrows, int numcols, const char * statement);

    int _numdofs;

    EnvironmentBasePtr _esEnv;
    RobotBasePtr _esRobot;
    std::vector<dReal> _lowerLimit, _upperLimit;

    std::stringstream _outputstream;

    boost::shared_ptr<ESParameters> _parameters;

    std::vector<ContactRegion> _contact_regions;

    std::map< int, ContactRegion > nearest_contact_regions; // <manips group, contact region>
    std::map< int, ContactManipGroup > contact_manips_group; // <group index, <manip name, contact-consistent transform, manip of (manip name) in the wp index belongs to this group> >
    std::map< size_t, std::map<string,int> > waypoint_manip_group_map; // <waypoint index, <manip_name,contact consistent transform> >

    std::vector<int> start_contact_group_index;
    std::vector<int> goal_contact_group_index;

    string _strRobotName; // name of the active robot

    Vector cogtarg;
    Vector curcog;
    Balance balance_checker;

    std::map<size_t,bool> stable_waypoint;

    std::map<size_t, std::map<string,int> > contact_consistent_region; // <waypoint index, <manip_name,cc manip position> >
    std::vector< std::pair<string,Vector> > contact_consistent_manip_translation; // store the contact consistent point/translation in each iteration, index: contact region index, <manip name, position of the link>

    NEWMAT::Matrix _Jp;
    NEWMAT::Matrix _Jp0;

    NEWMAT::Matrix _Jr;
    NEWMAT::Matrix _Jr0;

    NEWMAT::Matrix _Jr_proper;

    NEWMAT::Matrix _Jr_quat;

    NEWMAT::Matrix _tooltm;
    TransformMatrix _TMtool;


    NEWMAT::Matrix _tasktm;
    NEWMAT::Matrix _E_rpy;
    NEWMAT::Matrix _E_rpy_inv;

    TransformMatrix _TMtask;

    dReal _psi, _theta, _phi;
    dReal Cphi,Ctheta,Cpsi,Sphi,Stheta,Spsi;

    NEWMAT::Matrix JOA;
    NEWMAT::Matrix JOAplus;
    NEWMAT::ColumnVector doa;
    
    NEWMAT::Matrix JZ;
    NEWMAT::Matrix JRPY;

    NEWMAT::Matrix JZRPY;
    NEWMAT::Matrix JZRPYplus;
    NEWMAT::ColumnVector dzrpy;

    NEWMAT::Matrix JXY;
    NEWMAT::Matrix JXYplus;
    NEWMAT::ColumnVector dxy;

    NEWMAT::Matrix JXYZRPY;
    NEWMAT::Matrix JXYZRPYplus;
    NEWMAT::ColumnVector dxyzrpy;
    
    NEWMAT::Matrix JCOG;
    NEWMAT::Matrix JCOGplus;
    NEWMAT::ColumnVector dcog;

    NEWMAT::Matrix JPC;
    NEWMAT::Matrix JPCplus;
    NEWMAT::ColumnVector dpc;
    
    NEWMAT::SymmetricMatrix Moa;
    NEWMAT::SymmetricMatrix Moainv;

    NEWMAT::SymmetricMatrix Mzrpy;
    NEWMAT::SymmetricMatrix Mzrpyinv;

    NEWMAT::SymmetricMatrix Mxy;
    NEWMAT::SymmetricMatrix Mxyinv;

    NEWMAT::SymmetricMatrix Mxyzrpy;
    NEWMAT::SymmetricMatrix Mxyzrpyinv;

    NEWMAT::SymmetricMatrix Mcog;
    NEWMAT::SymmetricMatrix Mcoginv;

    NEWMAT::SymmetricMatrix Mpc;
    NEWMAT::SymmetricMatrix Mpcinv;

    dReal cxy = 1;
    dReal czrpy = 1;
    dReal ccog = 1;
    dReal coa = 1;
    dReal cpc = 1;

    dReal ce = 1;


    NEWMAT::DiagonalMatrix Regoa;
    NEWMAT::DiagonalMatrix Regxy;
    NEWMAT::DiagonalMatrix Regzrpy;
    NEWMAT::DiagonalMatrix Regxyzrpy;
    NEWMAT::DiagonalMatrix Regcog;
    NEWMAT::DiagonalMatrix Regpc;

    NEWMAT::Matrix* JHP;
    NEWMAT::Matrix* JHPplus;
    NEWMAT::ColumnVector* dhp;

    NEWMAT::Matrix JM;
    NEWMAT::Matrix JMplus;
    NEWMAT::ColumnVector dm;
    NEWMAT::SymmetricMatrix Mm;
    NEWMAT::SymmetricMatrix Mminv;
    NEWMAT::DiagonalMatrix Regm;

    NEWMAT::ColumnVector balance_step;
    NEWMAT::ColumnVector oa_step;
    NEWMAT::ColumnVector zrpy_step;
    NEWMAT::ColumnVector xy_step;
    NEWMAT::ColumnVector xyzrpy_step;
    NEWMAT::ColumnVector pc_step;
    NEWMAT::ColumnVector m_step;
    NEWMAT::ColumnVector step;
    
    Transform l_arm_manip_transform_offset;
    Transform r_arm_manip_transform_offset;
    Transform l_leg_manip_transform_offset;
    Transform r_leg_manip_transform_offset;
    
    //used in transform difference
    Transform _tmtemp;
    dReal _sumsqr,_sumsqr2;
    Vector _q1, _q2;

    //used in QuatToRPY
    dReal a,b,c,d;
    static Vector RPYIdentityOffsets[8];
    dReal _min_dist;
    Vector _temp_vec;
    dReal _temp_dist;

    //for inverse conditioning
    NEWMAT::DiagonalMatrix _S;
    NEWMAT::Matrix _V;

    dReal epsilon = 0.005; //error tolerance for manipulator pose constraint
    dReal xy_error;
    dReal z_error;
    dReal rpy_error;

    bool bInCollision;

    bool bLimit;
    std::vector< std::vector<int> > badjointinds;

    std::map<string,int> GetManipIndex;

};


#endif