#ifndef  ELASTICSTRIPS_H
#define  ELASTICSTRIPS_H

#include <cdd/setoper.h>
#include <cdd/cdd.h>

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
				boost::shared_ptr< NEWMAT::Matrix > J;
				boost::shared_ptr< NEWMAT::Matrix > Jplus;
		};

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


class ElasticStrips : ModuleBase
{
  public:
    ElasticStrips(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("RunElasticStrips",boost::bind(&ElasticStrips::RunElasticStrips,this,_1,_2),
                        "Run the Elastic Strips Planner");

        _esEnv = penv;
    }
    virtual ~ElasticStrips() {}

    int RunElasticStrips(ostream& sout, istream& sinput);
    bool InitPlan(RobotBasePtr robot, PlannerParametersConstPtr params);
    OpenRAVE::PlannerStatus PlanPath(TrajectoryBasePtr ptraj);
    // bool SmoothTrajectory(TrajectoryBasePtr ptraj);
    virtual PlannerParametersConstPtr GetParameters() const {return _parameters;}
    void FindContactRegions();
    void FindContactConsistentManipTranslation();
    Transform ForwardKinematics(std::vector<dReal> qs,string link_name);

    void UpdateZandXYJacobian();
    void UpdateCOGJacobian();
    void UpdateOAJacobianandStep(Transform taskframe_in);
    void UpdatePCJacobian();

    void UpdateZandXYStep();
    void UpdateCOGStep();
    void UpdatePCStep();

    NEWMAT::ColumnVector CalculateStep();

  private:
    int _numdofs;

    EnvironmentBasePtr _esEnv;
    RobotBasePtr _esRobot;
    std::vector<dReal> _lowerLimit, _upperLimit;

    PriorityTree _pritority_tree;

    std::stringstream _outputstream;

    boost::shared_ptr<ESParameters> _parameters;

    Vector cogtarg;
    Vector curcog;
    Balance balance_checker;

    std::map<size_t, std::map<string,int> > contact_consistent_region; // <waypoint index, <manip_name,cc manip position> >
    std::vector< std::pair<string,Vector> > contact_consistent_manip_translation; // store the contact consistent point/translation in each iteration

    NEWMAT::Matrix Jp;
    NEWMAT::Matrix Jp0;

    NEWMAT::Matrix J;
    
    NEWMAT::Matrix JOA;
    NEWMAT::Matrix JOAplus;
    NEWMAT::ColumnVector doa;
    
    NEWMAT::Matrix JZ;
    NEWMAT::Matrix JZplus;
    NEWMAT::Matrix JXY;
    NEWMAT::Matrix JXYplus;
    NEWMAT::ColumnVector dxy;
    NEWMAT::ColumnVector dz;

    NEWMAT::Matrix JCOG;
    NEWMAT::Matrix JCOGplus;
    NEWMAT::ColumnVector dcog;

    NEWMAT::Matrix JPC;
    NEWMAT::Matrix JPCplus;
    NEWMAT::ColumnVector dpc;
    
    NEWMAT::ColumnVector dx;
    NEWMAT::ColumnVector dx_trans;
    NEWMAT::ColumnVector step;
    NEWMAT::ColumnVector obstacleavoidancestep;

    NEWMAT::SymmetricMatrix M;
    NEWMAT::SymmetricMatrix Minv;
    
    NEWMAT::SymmetricMatrix Mcog;
    NEWMAT::SymmetricMatrix Mcoginv;
    
    NEWMAT::SymmetricMatrix Moa;
    NEWMAT::SymmetricMatrix Moainv;

    NEWMAT::SymmetricMatrix Mz;
    NEWMAT::SymmetricMatrix Mzinv;

    NEWMAT::SymmetricMatrix Mxy;
    NEWMAT::SymmetricMatrix Mxyinv;

    NEWMAT::SymmetricMatrix Mpc;
    NEWMAT::SymmetricMatrix Mpcinv;


    NEWMAT::ColumnVector step;

    NEWMAT::DiagonalMatrix Regxy;
    NEWMAT::DiagonalMatrix Regz;
    NEWMAT::DiagonalMatrix Regcog;
    NEWMAT::DiagonalMatrix Regoa;
    NEWMAT::DiagonalMatrix Regpc;
    
    // NEWMAT::SymmetricMatrix Mbalperp;
    // NEWMAT::SymmetricMatrix Mbalperpinv;
};


#endif