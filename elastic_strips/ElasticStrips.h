#ifndef  ELASTICSTRIPS_H
#define  ELASTICSTRIPS_H

#include <cdd/setoper.h>
#include <cdd/cdd.h>

class PriorityNode
{
    public:
        PriorityNode(string node_name, string parent_name)
        {
            name = node_name;
            parent = parent_name;
        }

        string name;
        string parent;
        // boost::shared_ptr< NEWMAT::Matrix > J;
        // boost::shared_ptr< NEWMAT::Matrix > Jplus;
        // boost::shared_ptr< NEWMAT::ColumnVector > v;
        NEWMAT::Matrix* J;
        NEWMAT::Matrix* Jplus;
        NEWMAT::ColumnVector* v;
};

class PriorityTree
{
	public:
		PriorityTree()
		{
			// PriorityNode JointLimit("JointLimit","JointLimit");
			// data.insert(std::pair<string,PriorityNode>(JointLimit.name,JointLimit));

			PriorityNode Z("ZRPY","ZRPY");
			data.insert(std::pair<string,PriorityNode>(Z.name,Z));

			PriorityNode XY("XY","ZRPY");
			data.insert(std::pair<string,PriorityNode>(XY.name,XY));

			PriorityNode ObstacleAvoidance("ObsAvoidance","ZRPY");
			data.insert(std::pair<string,PriorityNode>(ObstacleAvoidance.name,ObstacleAvoidance));

			PriorityNode Balance("Balance","ObsAvoidance");
			data.insert(std::pair<string,PriorityNode>(Balance.name,Balance));

			PriorityNode Posture("Posture","Balance");
			data.insert(std::pair<string,PriorityNode>(Posture.name,Posture));
		}

		bool ConstructSubtree(std::vector<string> node_name)
		{
			// Z always exists, so needless to consider the situation with multiple tree top.
			std::map<string,PriorityNode> subtree;
			for(std::vector<string>::iterator it = node_name.begin(); it != node_name.end(); it++)
			{
				subtree.insert((*data.find(*it)));
			}

			for(std::map<string,PriorityNode>::iterator it = subtree.begin(); it != subtree.end(); it++)
			{
				string curr_it_node_name = it->second.name;
				string parent_name = it->second.parent;
				while(subtree.count(parent_name) == 0)
				{
                    if(parent_name == curr_it_node_name)
					{
						cerr<<"ERROR: Failed to generate subtree."<<endl;
						return false;
					}
					curr_it_node_name = parent_name;
					parent_name = data.find(curr_it_node_name)->second.parent;
				}

				it->second.parent = parent_name;

			}

            data = subtree;

			return true;
		}

		std::map<string,PriorityNode> data;

};


class ElasticStrips : public ModuleBase
{
  public:
    ElasticStrips(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("RunElasticStrips",boost::bind(&ElasticStrips::RunElasticStrips,this,_1,_2),
                        "Run the Elastic Strips Planner");

        _priority_tree = PriorityTree();

    }
    virtual ~ElasticStrips() {}

    int RunElasticStrips(ostream& sout, istream& sinput);

  private:

    void SetActiveRobots(const std::vector<RobotBasePtr >& robots);

    void InitPlan(boost::shared_ptr<ESParameters> params);
    OpenRAVE::PlannerStatus PlanPath(TrajectoryBasePtr ptraj);
    void FindContactRegions();
    void FindContactConsistentManipTranslation(TrajectoryBasePtr ptraj);
    Transform ForwardKinematics(std::vector<dReal> qs,string link_name);
    dReal TransformDifference(dReal * dx,Transform tm_ref, Transform tm_targ);

    void GetRepulsiveVector(Vector& repulsive_vector, std::multimap<string,Vector>::iterator& control_point);

    void UpdateZRPYandXYJacobianandStep(Transform taskframe_in, int config_index);
    void UpdateCOGJacobianandStep(Transform taskframe_in);
    void UpdateOAJacobianandStep(Transform taskframe_in);
    void UpdatePCJacobianandStep(Transform taskframe_in);

    void QuatToRPY(Transform tm, dReal& psi, dReal& theta, dReal& phi);
    int invConditioningBound(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed);
    void RemoveBadJointJacobianCols(NEWMAT::Matrix& J);

    NEWMAT::ColumnVector CalculateStep();
    void PrintMatrix(dReal* pMatrix, int numrows, int numcols, const char * statement);

    int _numdofs;

    EnvironmentBasePtr _esEnv;
    RobotBasePtr _esRobot;
    std::vector<dReal> _lowerLimit, _upperLimit;

    PriorityTree _priority_tree;

    std::stringstream _outputstream;

    boost::shared_ptr<ESParameters> _parameters;

    string _strRobotName; ///< name of the active robot

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

    NEWMAT::SymmetricMatrix Mcog;
    NEWMAT::SymmetricMatrix Mcoginv;

    NEWMAT::SymmetricMatrix Mpc;
    NEWMAT::SymmetricMatrix Mpcinv;

    dReal cxy = 0.01;
    dReal czrpy = 0.01;
    dReal ccog = 0.01;
    dReal coa = 0.01;
    dReal cpc = 0.01;

    NEWMAT::ColumnVector step;


    NEWMAT::DiagonalMatrix Regoa;
    NEWMAT::DiagonalMatrix Regxy;
    NEWMAT::DiagonalMatrix Regzrpy;
    NEWMAT::DiagonalMatrix Regcog;
    NEWMAT::DiagonalMatrix Regpc;
    
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

    dReal epsilon = 0.001; //error tolerance for manipulator pose constraint
    dReal xy_error;
    // dReal zrpy_error;

    bool bInCollision;

    bool bLimit;
    std::vector<int> badjointinds;

    std::map<string,int> GetManipIndex;

};


#endif