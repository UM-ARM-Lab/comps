/* Copyright (c) 2015 Worcester Polytechnic Institute
   Author: Yu-Chi Lin <ylin5@wpi.edu>

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of Intel Corporation nor Carnegie Mellon University,
       nor the names of their contributors, may be used to endorse or
       promote products derived from this software without specific prior
       written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/** \file cbirrtmain.cpp
    \brief Registers cbirrt problem and cbirrt planner with openrave.
 */

/*! \mainpage CBiRRT Openrave Plugin
 *
 *
 * The Constrained BiDirectional Rapidly-exploring Random Tree (cbirrt) plugin contains the cbirrt problem, which parses commands from python or matlab and
 * calls the cbirrt planner. The cbirrt planner is a bidirectional sampling-based rrt which plans
 * on constraint manifolds induced by pose constraints as well as other constraints. This version of the software implements
 * pose constraints as Task Space Region (TSR) Chains. Pose constraints can apply to the start, goal, or entire path.
 * The planner is also set up to work with balance constraints.
 */

#include "stdafx.h"

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "elasticstrips" ) {
        return InterfaceBasePtr(new ElasticStrips(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("ElasticStrips");
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

int ElasticStrips::RunElasticStrips(ostream& sout, istream& sinput)
{
    // Lock environment mutex
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());

    boost::shared_ptr<ESParameters> params;
    params.reset(new ESParameters());

    RAVELOG_DEBUG("Starting Elastic Strips...\n");
    RAVELOG_INFO("Checking ravelog_info\n");

    // Parameters to populate from sinput
    dReal temp_r;                      // Used in manip
    bool bGetTime = false;             // from gettime

    TransformMatrix temp_tf_matrix; // Used in maniptm
    Transform temp_tf;              // Used in maniptm

    Vector cogtarg(0, 0, 0);
    std::vector<string> supportlinks;   // from supportlinks
    Vector polyscale(1.0, 1.0, 1.0);    // from polyscale
    Vector polytrans(0, 0, 0);          // from polytrans
    std::vector<string> support_manips; // from support
    std::vector<dReal> support_mus;     // from support
    Vector gravity(0.0, 0.0, -9.8);     // from gravity

    int num_waypoints;
    dReal temp_q;

    // Populated from info in supportlinks
    std::vector<dReal> supportpolyx;
    std::vector<dReal> supportpolyy;

    // Actual parameter vector directly passed to GeneralIK

    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
    ptraj->Init(robot->GetActiveConfigurationSpecification());

    // Command string holds the current command
    string cmd;

    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;

        if(stricmp(cmd.c_str(), "nummanips") == 0) {
            // Number of manipulators whose transforms are specified.
            // Seems like this always agrees with the number of "maniptm" commands?
            sinput >> params->_num_manips;
        }
        else if(stricmp(cmd.c_str(), "manips") == 0) {
            for(int i = 0; i < params->_num_manips; i++)
            {
                sinput >> temp_r;
                params->_targmanips.push_back(temp_r);    
            }
        }
        else if(stricmp(cmd.c_str(), "gettime") == 0) {
            // Whether the command's output should include the time.
            // If included it appears as the last space-separated number in the output string.
            bGetTime = true;
        }
        else if(stricmp(cmd.c_str(), "trajectory") == 0) {
            // This is mandatory for the elastic strips to work.
            sinput >> num_waypoints;
            std::vector<dReal> temp_waypoint(temp_r);
            int temp_index;
            for(int i = 0; i < num_waypoints; i++)
            {
                sinput >> temp_index;
                for(int j = 0; j < temp_r; j++)
                {
                    sinput >> temp_q;
                    temp_waypoint[j] = temp_q;
                }
                ptraj->Insert(temp_index,temp_waypoint,robot->GetActiveConfigurationSpecification());
            }
        }
        else if(stricmp(cmd.c_str(), "desiredmanippose") == 0) {
            // Specify the desired pose of each configuration in the trajectory.
            // Index, Number of specified pose in the configuration, The transformation.
            int temp_index;
            int numspecifiedmanip;
            int temp_link_index;
            sinput >> temp_index;
            sinput >> numspecifiedmanip;

            std::map<string,Transform> temp_manip_pose;
            for(int i = 0; i < numspecifiedmanip; i++)
            {
                sinput >> temp_link_index;
                sinput >> temp_tf_matrix.m[0];
                sinput >> temp_tf_matrix.m[4];
                sinput >> temp_tf_matrix.m[8];
                sinput >> temp_tf_matrix.m[1];
                sinput >> temp_tf_matrix.m[5];
                sinput >> temp_tf_matrix.m[9];
                sinput >> temp_tf_matrix.m[2];
                sinput >> temp_tf_matrix.m[6];
                sinput >> temp_tf_matrix.m[10];
                sinput >> temp_tf_matrix.trans.x;
                sinput >> temp_tf_matrix.trans.y;
                sinput >> temp_tf_matrix.trans.z;
                temp_tf = Transform(temp_tf_matrix);

                temp_manip_pose.insert(std::pair<string,Transform>(robot->GetLinks()[temp_link_index]->GetName(), temp_tf));
            }

            params->_desired_manip_pose.insert(std::pair<size_t, std::map<string,Transform> >(temp_index,temp_manip_pose));

        }
        else if(stricmp(cmd.c_str(), "checkcollision") == 0) {
            params->bOBSTACLE_AVOIDANCE = true;
            int numcheckcollisionlinks;
            string tempstring;
            sinput >> numcheckcollisionlinks;
            for(int i = 0; i < numcheckcollisionlinks; i++)
            {
                sinput >> tempstring;
                params->_control_points.insert(std::pair<string,Vector>(tempstring, robot->GetLink(tempstring)->GetCOMOffset()));
            }
        }
        else if(stricmp(cmd.c_str(), "checkselfcollision") == 0) {
            params->bOBSTACLE_AVOIDANCE = true;
            int numcheckselfcollisionlinkpairs;
            string tempstring_1;
            string tempstring_2;
            sinput >> numcheckselfcollisionlinkpairs;
            for(int i = 0; i < numcheckselfcollisionlinkpairs; i++)
            {
                sinput >> tempstring_1;
                sinput >> tempstring_2;
                params->_self_collision_checking_pairs.push_back(std::pair<string,string>(tempstring_1,tempstring_2));
            }
        }
        else if(stricmp(cmd.c_str(), "obstacles") == 0) {
            int numobstacles;
            string tempstring;
            sinput >> numobstacles;
            for(int i = 0; i < numobstacles; i++)
            {
                sinput >> tempstring;
                std::vector<KinBodyPtr> bodies;
                GetEnv()->GetBodies(bodies);
                for(int j = 0; j < bodies.size(); j++)
                {
                    if(stricmp(tempstring.c_str(),bodies[j]->GetName().c_str()) == 0)
                    {
                        params->_esObstacle.push_back(bodies[j]);
                        break;
                    }
                }
            }
        }
        else if(stricmp(cmd.c_str(), "posturecontrol") == 0) {
            // define link relative transform respect to the robot base.
            // (@{LINK_NAME},${LINK_TRANSFORM})
            params->bPOSTURE_CONTROL = true;
            int numposturecontrol;
            string temp_link;
            sinput >> numposturecontrol;

            for(int i = 0; i < numposturecontrol; i++)
            {
                sinput >> temp_link;

                sinput >> temp_tf_matrix.m[0];
                sinput >> temp_tf_matrix.m[4];
                sinput >> temp_tf_matrix.m[8];
                sinput >> temp_tf_matrix.m[1];
                sinput >> temp_tf_matrix.m[5];
                sinput >> temp_tf_matrix.m[9];
                sinput >> temp_tf_matrix.m[2];
                sinput >> temp_tf_matrix.m[6];
                sinput >> temp_tf_matrix.m[10];
                sinput >> temp_tf_matrix.trans.x;
                sinput >> temp_tf_matrix.trans.y;
                sinput >> temp_tf_matrix.trans.z;
                temp_tf = Transform(temp_tf_matrix);

                params->_posture_control.insert(std::pair<string,Transform>(temp_link,temp_tf));

            }
        }
        else if(stricmp(cmd.c_str(), "supportlinks") == 0){
            // The links to use for support-polygon stability checking. Format is
            // "supportlinks {num_links} {link_name}..."
            params->balance_mode = BALANCE_SUPPORT_POLYGON;
            int numsupportlinks;
            string tempstring;
            sinput >> numsupportlinks;
            for(int i =0; i < numsupportlinks; i++)
            {
                sinput >> tempstring;
                supportlinks.push_back(tempstring);
            }
        }
        else if(stricmp(cmd.c_str(), "polyscale") == 0){
            // 3d vector to scale the support polygon for support-polygon stability checking. Intended to make it
            // possible to specify more conservative constraints (disallow barely-balanced situations).
            sinput >> polyscale.x;
            sinput >> polyscale.y;
            sinput >> polyscale.z;
        }
        else if(stricmp(cmd.c_str(), "polytrans") == 0){
            // 3d vector to translate the support polygon for support-polygon stability checking.
            sinput >> polytrans.x;
            sinput >> polytrans.y;
            sinput >> polytrans.z;
        }
        else if(stricmp(cmd.c_str(), "support") == 0){
            // Support specifier for GIWC stability checking. May be provided more than once. Format is
            // "support {manip_index} {friction_coeff}".
            string tempstr;
            sinput >> tempstr; support_manips.push_back(tempstr);
            sinput >> temp_r; support_mus.push_back(temp_r);
            params->balance_mode = BALANCE_GIWC;
        }
        else if(stricmp(cmd.c_str(), "gravity") == 0){
            sinput >> gravity.x;
            sinput >> gravity.y;
            sinput >> gravity.z;
        }
        else break;

        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            return false;
        }
    }

    if(params->balance_mode == BALANCE_SUPPORT_POLYGON)
    {
        if(supportlinks.size() == 0)
        {
            RAVELOG_INFO("ERROR: Must specify support links to do balancing\n");
            return false;
        }

        Balance b(robot, supportlinks, polyscale, polytrans);
        balance_checker = b;

    }
    else if(params->balance_mode == BALANCE_GIWC)
    {
        Balance b(robot, gravity, support_manips, support_mus);
        balance_checker = b;
    }


    unsigned long starttime = timeGetTime();


    if(!InitPlan(robot,params))
    {
        RAVELOG_INFO("InitPlan failed\n");
        return false;
    }

    // std::vector<dReal> qResult(robot->GetActiveDOF());
    // boost::shared_ptr<std::vector<dReal> > pqResult(new std::vector<dReal> );
    
    if(PlanPath(ptraj) == PS_HasSolution)
    {
        // qResult = *pqResult.get();
        int timetaken = timeGetTime() - starttime;
        // for(int i = 0; i < qResult.size(); i++)
        // {
        //     sout << qResult[i] << " ";
        // }
        if(bGetTime)
            sout << timetaken << " ";

        RAVELOG_INFO("Elastic Strips Finished! (%d ms)\n",timetaken);
        return true;
    }
    else
    {
        int timetaken = timeGetTime() - starttime;
        
        if(bGetTime)
            sout << timetaken << " ";
        RAVELOG_INFO("Elastic Strips Failed to Converge. (%d ms)\n",timetaken);
        return true;
    }
}

Transform ElasticStrips::ForwardKinematics(std::vector<dReal> qs,string link_name)
{
    _esRobot.SetDOFValues(qs);
    return _esRobot.GetLink(link_name)->GetTransform();
}

void ElasticStrips::FindContactRegions()
{
    for(map<size_t, map<string,Transform> >::iterator dmp_it = _desired_manip_pose.begin(); dmp_it != _desired_manip_pose.end(); dmp_it++)
    {
        map<string,int> temp_contact_region;
        for(map<string,Transform>::iterator p_it = dmp_it->second.begin(); p_it != dmp_it->second.end(); p_it++)
        {
            bool matched_contact_region_found = false;
            RaveVector desired_trans = p_it->second.trans;
            for(std::vector< std::pair<string,Vector> >::iterator ccmt_it = contact_consistent_manip_translation.begin(); ccmt_it != contact_consistent_manip_translation.end(); ccmt_it++)
            {
                string temp_link_name = ccmt_it->first;
                Vector temp_contact_manip_trans = ccmt_it->second;
                if(temp_link_name == p_it->first)
                {
                    if(fabs(desired_trans.x - temp_contact_manip_trans.x)+
                       fabs(desired_trans.y - temp_contact_manip_trans.y)+
                       fabs(desired_trans.z - temp_contact_manip_trans.z) < 0.001)
                    {
                        map<size_t, map<string,Transform> >::iterator p_dmp_it = std::prev(dmp_it);
                        int contact_region_index = ccmt_it - contact_consistent_manip_translation.begin();
                        int p_contact_region_index = contact_consistent_region.find(p_dmp_it->first)->second.find(p_it->first)->second;
                        if(contact_region_index == p_contact_region_index)
                        {
                            temp_contact_region.insert(std::pair<string,int>(p_it->first, contact_region_index));
                            matched_contact_region_found = true;
                            break;
                        }
                    }
                }
            }

            if(!matched_contact_region_found)
            {
                temp_contact_region.insert(std::pair<string,int>(p_it->first, contact_consistent_manip_translation.size()));
                contact_consistent_manip_translation.push_back(std::pair<string,Vector>(p_it->first,desired_trans));
            }
        }

        contact_consistent_region.insert(std::pair<size_t, map<string,Transform> >(dmp_it->first, temp_contact_region));
    }
}

void ElasticStrips::FindContactConsistentManipTranslation()
{
    std::vector< std::vector<Vector> > temp_contact_consistent_manip_translations(contact_consistent_manip_translation.size()); 
    for(int w = 0; w < ftraj->GetNumWaypoints(); w++)
    {
        vector<dReal> qt;
        ftraj->GetWaypoint(w,qt);

        std::map<string,int> contact_regions = contact_consistent_region.find(w)->second;
        
        for(std::map<string,int>::iterator cr_it = contact_regions.begin(); cr_it != contact_regions.end(); cr_it++)
        {
            temp_contact_consistent_manip_translations.at(cr_it->second).push_back(ForwardKinematics(qt,cr_it->first).trans());
        }
    }

    for(std::vector< std::vector<Vector> > tccmt_it = temp_contact_consistent_manip_translations.begin(); tccmt_it != temp_contact_consistent_manip_translations.end(); tccmt_it++)
    {
        Vector v(0,0,0);
        int contact_region_index = tccmt_it - temp_contact_consistent_manip_translations.begin();
        for(std::vector<Vector> it = tccmt_it->begin(); it != tccmt->end(); it++)
        {
            v.x += it->x;
            v.y += it->y;
            v.z += it->z;
        }
        v.x /= tccmt_it->size();
        v.y /= tccmt_it->size();
        v.z /= tccmt_it->size();

        contact_consistent_manip_translation.at(contact_region_index).second = v;
    }
}


void ElasticStrips::InitPlan(RobotBasePtr robot, PlannerParametersConstPtr params)
{
    //0. parameter initialization
    _esRobot = robot;
    _numdofs = _esRobot.GetActiveDOF();

    //1. receive command
    RobotBase::ManipulatorConstPtr activemanip_p = _pRobot->GetActiveManipulator();

    dReal* qResult = &(*result.get())[0];
    const dReal* pFreeParameters = &vFreeParameters[0];
    solutionpath.resize(0);
    // _targmanips.resize(0);
    // _targtms.resize(0);
    movementlimit = INF;

    bool bsuccess = true;
    _esRobot->GetActiveDOFLimits(_lowerLimit,_upperLimit);

    _numdofs = _esRobot->GetActiveDOF();

    //3. dynamically generate jacobian trees
    PriorityTree priority_tree();
    std::map<string,PriorityNode> working_subtree;
    std::vector<string> priority_tree_node_name;

    // priority_tree_node_name.push_back("JointLimit");
    priority_tree_node_name.push_back("Z");
    priority_tree_node_name.push_back("XY");
    if(params->bOBSTACLE_AVOIDANCE) priority_tree_node_name.push_back("ObsAvoidance");
    if(params->balance_mode != BALANCE_NONE) priority_tree_node_name.push_back("Balance");
    if(params->bPOSTURE_CONTROL) priority_tree_node_name.push_back("Posture");

    priority_tree.ConstructSubtree(priority_tree_node_name,working_subtree);

    _priority_tree = priority_tree;


    _parameters = params;

    //Group the contact regions
    FindContactRegions();

    Jp.ReSize(3,_numdofs);
    Jp0.ReSize(3,_numdofs);

}



void ElasticStrips::UpdateZandXYJacobian()
{
    // XY
    JXY.ReSize(2*desired_manip_pose.size(),_numdofs);
    JXYplus.ReSize(_numdofs,2*desired_manip_pose.size());
    Mxy.ReSize(2*desired_manip_pose.size());
    dxy.ReSize(2*desired_manip_pose.size());
    Regxy.ReSize(2*desired_manip_pose.size());
    Regxy = 0.0001;
    _pritority_tree.find("XY")->second.J = &JXY;
    _pritority_tree.find("XY")->second.Jplus = &JXYplus;

    // Z
    JZ.ReSize(desired_manip_pose.size(),_numdofs);
    JZplus.ReSize(_numdofs,desired_manip_pose.size());
    Mz.ReSize(desired_manip_pose.size());
    dz.ReSize(desired_manip_pose.size());
    Regz.ReSize(desired_manip_pose.size());
    Regz = 0.0001;
    _pritority_tree.find("Z")->second.J = &JZ;
    _pritority_tree.find("Z")->second.Jplus = &JZplus;

    map<string,Transform> desired_manip_pose = _parameters->_desired_manip_pose.find(w)->second;
    for(map<string,Transform>::iterator dmp_it = desired_manip_pose.begin(); dmp_it != desired_manip_pose.end(); dmp_it++)
    {
        int i = dmp_it - desired_manip_pose.begin();
        string link_name = dmp_it->first;

        // RaveTransformMatrix<dReal> desired_link_pose(dmp_it->second);
        RaveTransformMatrix<dReal> desired_link_pose_rot = geometry::matrixFromQuat(dmp_it->second.rot);
        RaveTransformMatrix<dReal> inverse_desired_link_pose_rot = desired_link_pose_rot.inverse();

        KinBody::LinkPtr target_link = _esRobot.GetLink(link_name);
        std::vector<dReal> temp;

        _esRobot->CalculateActiveJacobian(target_link->GetIndex(), target_link->GetTransform(), temp);
        memcpy(Jp0.Store(),&temp[0],temp.size()*sizeof(dReal));

        Jp = inverse_desired_link_pose_rot * Jp0;
        JXY.Rows(i*2+1,i*2+2) = Jp.Rows(1,2);
        JZ.Row(i+1) = Jp.Row(3);
    }

    Mxy = JXY*JXY.t() + Regxy;
    Mz = JZ*JZ.t() + Regz;

    invConditioningBound(10000,Mxy,Mxyinv);
    invConditioningBound(10000,Mz,Mzinv);

    JXYplus = JXY.t()*Mxyinv;
    JZplus = JZ.t()*Mzinv;

}

void ElasticStrips::UpdateCOGJacobian(Transform taskframe_in, Vector& center)
{
    JCOG = 0.0;

    center = Vector(0,0,0,0);
    dReal fTotalMass = 0;
    std::vector<KinBody::LinkPtr>::const_iterator itlink;

    _TMtask = TransformMatrix(taskframe_in.inverse());
    _tasktm(1,1) = _TMtask.m[0];        _tasktm(1,2) = _TMtask.m[1];        _tasktm(1,3) = _TMtask.m[2];
    _tasktm(2,1) = _TMtask.m[4];        _tasktm(2,2) = _TMtask.m[5];        _tasktm(2,3) = _TMtask.m[6];
    _tasktm(3,1) = _TMtask.m[8];        _tasktm(3,2) = _TMtask.m[9];        _tasktm(3,3) = _TMtask.m[10];


    std::vector<dReal> temp;
    FORIT(itlink, _esRobot->GetLinks())
    {
        _esRobot->CalculateActiveJacobian((*itlink)->GetIndex(), ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset()), temp);
        memcpy(Jp0.Store(),&temp[0],temp.size()*sizeof(dReal));

        Jp = _tasktm * Jp0;

        JCOG = JCOG + ((*itlink)->GetMass()*(Jp.Rows(1,2)));

        center += ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset() * (*itlink)->GetMass());
        fTotalMass += (*itlink)->GetMass();
    }

    if( fTotalMass > 0 )
        center /= fTotalMass;
   
    JCOG = JCOG/fTotalMass;

    Mcog = JCOG*JCOG.t() + Regcog;

    invConditioningBound(10000,Mcog,Mcoginv);
}

void ElasticStrips::GetRepulsiveVector(Vector& repulsive_vector, std::multimap<string,Vector>::iterator& control_point)
{
    dReal repulse_dist = 1000;
    dReal repulse_constant = -1;
    Transform link_transform = _pRobot->GetLink(control_point->first)->GetTransform();
    Vector control_point_global_position = link_transform*control_point->second;
    repulsive_vector = Vector(0,0,0);
    dReal shortest_dist = 100000000;

    //environment collision
    for(std::vector<KinBodyPtr>::iterator obs_it = _pObstacle.begin(); obs_it != _pObstacle.end(); obs_it++)
    {
        std::vector<KinBody::LinkPtr> ObstacleLink = (*obs_it)->GetLinks();
        for(std::vector<KinBody::LinkPtr>::iterator link_it = ObstacleLink.begin(); link_it != ObstacleLink.end(); link_it++)
        {
            if(_pEnvironment->CheckCollision(_pRobot->GetLink(control_point->first),(*link_it)))
            {
                std::vector<KinBody::Link::GeometryPtr> ObstacleGeometry = (*link_it)->GetGeometries();
                for(std::vector<KinBody::Link::GeometryPtr>::iterator geom_it = ObstacleGeometry.begin(); geom_it != ObstacleGeometry.end(); geom_it++)
                {
                    GeometryType obstacle_geometry_type = (*geom_it)->GetType();
                    RaveTransform<dReal> obstacle_geometry_transform = (*obs_it)->GetTransform() * (*link_it)->GetTransform() * (*geom_it)->GetTransform();
                    RaveTransformMatrix<dReal> obstacle_rot_matrix = geometry::matrixFromQuat(obstacle_geometry_transform.rot);
                    RaveTransformMatrix<dReal> inverse_obstacle_rot_matrix = obstacle_rot_matrix.inverse();
                    RaveVector<dReal> obstacle_translation = obstacle_geometry_transform.trans;
                    RaveVector<dReal> obstacle_frame_control_point_position = inverse_obstacle_rot_matrix * (control_point_global_position-obstacle_translation);
                    dReal dist_to_obstacle = 0;
                    RaveVector<dReal> repulsive_vector_component(0,0,0);
                    RaveVector<dReal> nearest_point(0,0,0);
                    if(obstacle_geometry_type == GT_Box)
                    {
                        Vector box_extents = (*geom_it)->GetBoxExtents();
                        if(obstacle_frame_control_point_position.x > box_extents.x/2)
                            nearest_point.x = box_extents.x/2;
                        else if(obstacle_frame_control_point_position.x < -box_extents.x/2)
                            nearest_point.x = -box_extents.x/2;
                        else
                            nearest_point.x = obstacle_frame_control_point_position.x;
                        
                        if(obstacle_frame_control_point_position.y > box_extents.y/2)
                            nearest_point.y = box_extents.y/2;
                        else if(obstacle_frame_control_point_position.y < -box_extents.y/2)
                            nearest_point.y = -box_extents.y/2;
                        else
                            nearest_point.y = obstacle_frame_control_point_position.y;

                        if(obstacle_frame_control_point_position.z > box_extents.z/2)
                            nearest_point.z = box_extents.z/2;
                        else if(obstacle_frame_control_point_position.z < -box_extents.z/2)
                            nearest_point.z = -box_extents.z/2;
                        else
                            nearest_point.z = obstacle_frame_control_point_position.z;

                        if(obstacle_frame_control_point_position.x != nearest_point.x ||
                           obstacle_frame_control_point_position.y != nearest_point.y ||
                           obstacle_frame_control_point_position.z != nearest_point.z)
                        {
                            repulsive_vector_component = obstacle_frame_control_point_position - nearest_point;
                            dist_to_obstacle = repulsive_vector_component.lengthsqr3();
                            repulsive_vector_component = (obstacle_rot_matrix*repulsive_vector_component).normalize3();
                        }
                        else
                        {
                            nearest_point = RaveVector<dReal>(0,0,0);
                            repulsive_vector_component = obstacle_frame_control_point_position - nearest_point;
                            repulsive_vector_component = (obstacle_rot_matrix*repulsive_vector_component).normalize3();
                            dist_to_obstacle = 0;
                        }

                    }
                    else if(obstacle_geometry_type == GT_Sphere)
                    {
                        repulsive_vector_component = control_point_global_position - obstacle_translation;
                        if(repulsive_vector_component.lengthsqr3() > (*geom_it)->GetSphereRadius())
                            dist_to_obstacle = repulsive_vector_component.lengthsqr3() - (*geom_it)->GetSphereRadius();
                        else
                            dist_to_obstacle = 0;
                        repulsive_vector_component = repulsive_vector_component.normalize3();
                    }
                    else if(obstacle_geometry_type == GT_Cylinder)
                    {
                        dReal cylinder_height = (*geom_it)->GetCylinderHeight();
                        dReal cylinder_radius = (*geom_it)->GetCylinderRadius();
                        //RaveVector<dReal> nearest_point(0,0,0);
                        dReal xy_dist_to_centroid = sqrt(pow(obstacle_frame_control_point_position.x,2) + pow(obstacle_frame_control_point_position.y,2));

                        if(xy_dist_to_centroid > cylinder_radius || fabs(obstacle_frame_control_point_position.z) > cylinder_height/2)
                        {
                            nearest_point.x = (cylinder_radius/xy_dist_to_centroid) * obstacle_frame_control_point_position.x;
                            nearest_point.y = (cylinder_radius/xy_dist_to_centroid) * obstacle_frame_control_point_position.y;

                            if(obstacle_frame_control_point_position.z > cylinder_height/2)
                                nearest_point.z = cylinder_height/2;
                            else if(obstacle_frame_control_point_position.z < -cylinder_height/2)
                                nearest_point.z = -cylinder_height/2;
                            else
                                nearest_point.z = obstacle_frame_control_point_position.z;

                            repulsive_vector_component = obstacle_frame_control_point_position - nearest_point;
                            dist_to_obstacle = repulsive_vector_component.lengthsqr3();                    
                        }
                        else
                        {
                            nearest_point = RaveVector<dReal>(0,0,0);
                            repulsive_vector_component = obstacle_frame_control_point_position - nearest_point;
                            dist_to_obstacle = 0;
                        }

                        repulsive_vector_component = (obstacle_rot_matrix*repulsive_vector_component).normalize3();
                    }

                    if(dist_to_obstacle < repulse_dist)
                        repulsive_vector = repulsive_vector + repulsive_vector_component;

                    if(dist_to_obstacle < shortest_dist)
                        shortest_dist = dist_to_obstacle;

                }

            }
        }
    }

    //self collision
    for(std::vector<std::pair<string,string> >::iterator sc_it = self_collision_checking_pairs.begin(); sc_it != self_collision_checking_pairs.end(); sc_it++)
    {
        string link_1 = (*sc_it).first;
        string link_2 = (*sc_it).second;
        if(control_point->first == link_1 || control_point->first == link_2)
        {
            if(_pEnvironment->CheckCollision(_pRobot->GetLink(link_1),_pRobot->GetLink(link_2)))
            {
                RaveVector<dReal> repulsive_vector_component(0,0,0);
                string other_link = (control_point->first == link_1) ? link_2 : link_1;
                RaveVector<dReal> other_link_centroid = _pRobot->GetLink(other_link)->GetTransform().trans;
                repulsive_vector_component = (control_point_global_position - other_link_centroid).normalize3();
                repulsive_vector = repulsive_vector + repulsive_vector_component;
                shortest_dist = 0;
            }
        }
    }

    if(repulsive_vector.lengthsqr3() != 0)
        repulsive_vector = repulse_constant * exp(-shortest_dist) * repulsive_vector * (1/repulsive_vector.lengthsqr3());

    //for each obstacle, find its geometry type
    //calculate distance between the control point and the obstacle
    //calculate the repulsive from each obstacle according to ther relative position and distance
    //sum the repulsive vector
}

void ElasticStrips::UpdateOAJacobianandStep(Transform taskframe_in)
{
    //calculate the velocity of each control point
    //calculate jacobian for each control point to generate the joint angular velocity
    //sum the joint angular velocity

    doa.ReSize(_numdofs);
    doa = 0;

    std::map<string,Vector> control_points_in_collision;
    std::map<string,Vector> control_point_repulsive_vector;
    std::map<string,NEWMAT::Matrix> control_point_jacobian;

    for(std::map<string,Vector>::iterator ctrl_it = _parameters->_control_points.begin(); ctrl_it != _parameters->_control_points.end(); ctrl_it++)
    {
        Vector repulsive_vector;
        GetRepulsiveVector(repulsive_vector, ctrl_it);

        if(repulsive_vector.lengthsqr3() != 0)
        {
            Jp.ReSize(3,_numdofs);

            dReal fTotalMass = 0;

            _TMtask = TransformMatrix(taskframe_in.inverse());
            _tasktm(1,1) = _TMtask.m[0];        _tasktm(1,2) = _TMtask.m[1];        _tasktm(1,3) = _TMtask.m[2];
            _tasktm(2,1) = _TMtask.m[4];        _tasktm(2,2) = _TMtask.m[5];        _tasktm(2,3) = _TMtask.m[6];
            _tasktm(3,1) = _TMtask.m[8];        _tasktm(3,2) = _TMtask.m[9];        _tasktm(3,3) = _TMtask.m[10];

            KinBody::LinkPtr target_link = _pRobot->GetLink(ctrl_it->first);
            std::vector<dReal> temp;

            _pRobot->CalculateActiveJacobian(target_link->GetIndex(), (target_link->GetTransform() * ctrl_it->second), temp);
            memcpy(Jp0.Store(),&temp[0],temp.size()*sizeof(dReal));

            Jp = _tasktm * Jp0;

            control_points_in_collision.insert(*ctrl_it);
            control_point_repulsive_vector.insert(std::pair<string,Vector>(ctrl_it->first,repulsive_vector));
            control_point_jacobian.insert(std::pair<string,NEWMAT::Matrix>(ctrl_it->first,Jtemp));
        }
    }

    if(!control_points_in_collision.empty())
    {
        Jp.ReSize(0,_numdofs);
        doa.ReSize(3*control_points_in_collision.size());

        int point_index = 0;

        //stack the repulsive vector and jacobian matrix
        for(std::map<string,Vector>::iterator ctrl_it = control_points_in_collision.begin(); ctrl_it != control_points_in_collision.end(); ctrl_it++)
        {
            Jp &= control_point_jacobian.find(ctrl_it->first)->second;
            doa(point_index*3+1) = control_point_repulsive_vector.find(ctrl_it->first)->second.x;
            doa(point_index*3+2) = control_point_repulsive_vector.find(ctrl_it->first)->second.y;
            doa(point_index*3+3) = control_point_repulsive_vector.find(ctrl_it->first)->second.z;
            point_index++;
        }

        JOA = Jp;

        Regoa.ReSize(JOA.Nrows());
        Regoa = 0.0001;
        Moa.ReSize(JOA.Nrows());
        Moainv.ReSize(JOA.Nrows());
        Moa << (JOA*JOA.t()) + Regoa;
        invConditioningBound(10000,Moa,Moainv);
        JOAplus.ReSize(_numdofs,3*control_points_in_collision.size());
        JOAplus = JOA.t()*Moainv;

        doa = 0.05 * doa / control_points_in_collision.size();
    }
    else
    {
        doa.ReSize(1);
        JOA.ReSize(1,_numdofs);
        JOAplus.ReSize(_numdofs,1);
        doa = 0.0;
        JOA = 0.0;
        JOAplus = 0.0;
    }

    Jp.ReSize(3,_numdofs);
    Jp0.ReSize(3,_numdofs);
}


void ElasticStrips::UpdateZandXYStep()
{
    cogtarg = Vector(0,0,0);
    
    map<string,Transform> desired_manip_pose = _parameters->_desired_manip_pose.find(w)->second;
    for(map<string,Transform>::iterator dmp_it = desired_manip_pose.begin(); dmp_it != desired_manip_pose.end(); dmp_it++)
    {
        int i = dmp_it - desired_manip_pose.begin();
        string link_name = dmp_it->first;
        Vector contact_consistent_point = contact_consistent_manip_translation.find(w)->second.find(link_name)->second;

        RaveVector<dReal> link_pose_trans = _esRobot->GetLink(link_name)->GetTransform().trans;
        RaveVector<dReal> translation_diff = link_pose_trans - contact_consistent_point;
        translation_diff = inverse_desired_link_pose_rot * translation_diff;

        //velocity, can be written as force
        dxy(i*2+1) = translation_diff.x;
        dxy(i*2+2) = translation_diff.y;
        dz(i+1) = translation_diff.z;

        //dirty code to decide cogtarg:(specialize to escher robot)
        if(link_name = 'l_leg' || link_name = 'r_leg')
        {
            cogtarg.x += contact_consistent_point.x;
            cogtarg.y += contact_consistent_point.y;
        }
        //************//
    }

    cogtarg.x /= 2.0;
    cogtarg.y /= 2.0;
}

void ElasticStrips::UpdateCOGStep()
{
    if(!CheckSupport(curcog))
    {
        dcog(1) = (curcog.x - cogtarg.x);
        dcog(2) = (curcog.y - cogtarg.y);
    }
    else
    {
        dcog(1) = 0;
        dcog(2) = 0;
    }
}



OpenRAVE::PlannerStatus ElasticStrips::PlanPath(TrajectoryBasePtr ptraj)
{
    OpenRAVE::PlannerStatus result = PS_HasSolution;

    // Initialize invaraint Jacobian dimension of each constriant.
    if(_parameters->bPOSTURE_CONTROL)
    {
        JPC.ReSize(6*_parameters->_posture_control.size(),_numdofs);
        JPCplus.ReSize(_numdofs,6*_parameters->_posture_control.size());
        Mpc.ReSize(6*_parameters->_posture_control.size());
        Regpc.ReSize(6*_parameters->_posture_control.size());
        Regpc = 0.0001;
        _pritority_tree.find("Posture")->second.J = &JPC;
        _pritority_tree.find("Posture")->second.Jplus = &JPCplus;
    }

    if(_parameters->balance_mode != BALANCE_NONE)
    {
        JCOG.ReSize(2,_numdofs);
        JCOGplus.ReSize(_numdofs,2);
        Mcog.ReSize(2);
        Regcog.ReSize(2);
        Regcog = 0.0001;
        _pritority_tree.find("Balance")->second.J = &JCOG;
        _pritority_tree.find("Balance")->second.Jplus = &JCOGplus;
    }
    

    TrajectoryBasePtr ftraj = RaveCreateTrajectory(GetEnv(),"");
    ftraj->Init(_esRobot->GetActiveConfigurationSpecification());
    *ftraj = *ptraj;
    
    TrajectoryBasePtr ntraj = RaveCreateTrajectory(GetEnv(),"");
           
    for(int k = 0; k < 200; k++) // modify the configuration
    {
        ntraj->Init(_esRobot->GetActiveConfigurationSpecification());

        // Calculate the contact consistent manipulator translation
        FindContactConsistentManipTranslation();

        for(int w = 0; w < ftraj->GetNumWaypoints(); w++) // for each configuration in the trjectory
        {
            // Get the initial configuration
            std::vector<dReal> qs(_numdofs); // the initial configuration
            ftraj->GetWaypoint(w,qs);

            // Calculate each Jacobian
            _esRobot->SetDOFValues(qs);

            // 1. Z
            // 2. XY
            UpdateZandXYJacobian();
            UpdateZandXYStep();

            // 3. Balance / COG
            if(_parameters->balance_mode != BALANCE_NONE)
            {
                UpdateCOGJacobian(Transform(),curcog);
                UpdateCOGStep();
            }
            
            // 4. Obstacle Avoidance
            if(_parameters->bOBSTACLE_AVOIDANCE)
            {
                UpdateOAJacobian(Transform());
            }
            
            // 5. Posture Control
            if(_parameters->bPOSTURE_CONTROL)
            {
                UpdatePCJacobian();
                UpdatePCStep();
            }

            // Calculate steps by iteration, and move the robot.
            // step = CalculateStep();

            ntraj->Insert(w,qs,_esRobot->GetActiveConfigurationSpecification());
        }

        *ftraj = *ntraj;
    }

    ptraj = ftraj;

    return result;
}