#ifndef  ESPARAMETERS_H
#define  ESPARAMETERS_H

/// class for passing parameters to cbirrt planner
class ESParameters
{
    public:
        ESParameters() :
            bOBSTACLE_AVOIDANCE(false), bPOSTURE_CONTROL(false), balance_mode(BALANCE_NONE), contact_region_radius(0.05)
        {

        }

        dReal contact_region_radius;

        std::map<string,Vector> _obstacle_list;

        int _num_manips;
        std::vector<int> _targmanips;
        std::vector<Transform> _targtms;

        std::vector<KinBodyPtr> _esObstacle;
        std::multimap<string,Vector> _control_points; //<link_name,local translation>
        std::vector<std::pair<string,string> > _self_collision_checking_pairs; //need another reading command from user input
        
        map<size_t, map<string,Transform> > _desired_manip_pose; //<traj index, <link name, link transform> >
        map<string,Transform> _posture_control; //<link name,link transform>

        bool bOBSTACLE_AVOIDANCE;
        bool bPOSTURE_CONTROL;
        BalanceMode balance_mode;
};

#endif