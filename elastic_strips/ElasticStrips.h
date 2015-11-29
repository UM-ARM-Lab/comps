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
			PriorityNode JointLimit("JointLimit","JointLimit");
			data.insert(std::pair<string,PriorityNode>(JointLimit.name,JointLimit));

			PriorityNode Z("Z","JointLimit");
			data.insert(std::pair<string,PriorityNode>(Z.name,Z));

			PriorityNode XY("XY","Z");
			data.insert(std::pair<string,PriorityNode>(XY.name,XY));

			PriorityNode ObstacleAvoidance("ObsAvoidance","Z");
			data.insert(std::pair<string,PriorityNode>(ObstacleAvoidance.name,ObstacleAvoidance));

			PriorityNode Balance("Balance","ObsAvoidance");
			data.insert(std::pair<string,PriorityNode>(Balance.name,Balance));

			PriorityNode Posture("Posture","Balance");
			data.insert(std::pair<string,PriorityNode>(Posture.name,Posture));
		}

		bool ConstructSubtree(std::vector<string> node_name, std::map<string,PriorityNode>& subtree)
		{
			//JointLimit always exists, so needless to consider the situation with multiple tree top.
			subtree.clear();
			for(std::vector<string>::iterator it = node_name.begin(); it != node_name.end(); it++)
			{
				subtree.insert((*data.find(*it)));
			}

			for(std::map<string,PriorityNode>::iterator it = subtree.begin(); it != subtree.end(); it++)
			{
				string curr_it_node_name = it->second.name;
				string parent_name = it->second.parent;
				while(subtree.count(parent_name) != 0)
				{
					if(parent_name == curr_it_node_name)
					{
						cerr<<"ERROR: Failed to generate subtree."<<endl;
						return false;
					}
					curr_it_node_name = parent_name;
					parent_name = data.find(curr_it_node_name)->second.name;
				}

				it->second.parent = parent_name;

			}

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

        _esEnv = penv;
        _priority_tree = PriorityTree();

    }
    virtual ~ElasticStrips() {}

    int main(const std::string& cmd);
    void SetActiveRobots(const std::vector<RobotBasePtr >& robots);

    int RunElasticStrips(ostream& sout, istream& sinput);
    void InitPlan(boost::shared_ptr<ESParameters> params);
    OpenRAVE::PlannerStatus PlanPath(TrajectoryBasePtr ptraj);
    // bool SmoothTrajectory(TrajectoryBasePtr ptraj);
    void FindContactRegions();
    void FindContactConsistentManipTranslation(TrajectoryBasePtr ptraj);
    Transform ForwardKinematics(std::vector<dReal> qs,string link_name);
    dReal TransformDifference(dReal * dx,Transform tm_ref, Transform tm_targ);

    void GetRepulsiveVector(Vector& repulsive_vector, std::multimap<string,Vector>::iterator& control_point);

    void UpdateZandXYJacobianandStep(Transform taskframe_in, int config_index);
    void UpdateCOGJacobianandStep(Transform taskframe_in);
    void UpdateOAJacobianandStep(Transform taskframe_in);
    void UpdatePCJacobianandStep(Transform taskframe_in);

    NEWMAT::ColumnVector CalculateStep();

  private:

    void QuatToRPY(Transform tm, dReal& psi, dReal& theta, dReal& phi);
    int invConditioningBound(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed);
    void RemoveBadJointJacobianCols(NEWMAT::Matrix& J);

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
    std::vector< std::pair<string,Vector> > contact_consistent_manip_translation; // store the contact consistent point/translation in each iteration, index: traj index, <link/endeffector name, position of the link>

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
    NEWMAT::Matrix JZplus;
    NEWMAT::ColumnVector dz;

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

    NEWMAT::SymmetricMatrix Mz;
    NEWMAT::SymmetricMatrix Mzinv;

    NEWMAT::SymmetricMatrix Mxy;
    NEWMAT::SymmetricMatrix Mxyinv;

    NEWMAT::SymmetricMatrix Mcog;
    NEWMAT::SymmetricMatrix Mcoginv;

    NEWMAT::SymmetricMatrix Mpc;
    NEWMAT::SymmetricMatrix Mpcinv;

    NEWMAT::ColumnVector step;


    NEWMAT::DiagonalMatrix Regoa;
    NEWMAT::DiagonalMatrix Regxy;
    NEWMAT::DiagonalMatrix Regz;
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

    bool bInCollision;

    bool bLimit;
    std::vector<int> badjointinds;

};


#endif