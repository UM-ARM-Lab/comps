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

/** \file ElasticStrips.cpp

 *
 *
 * The Constrained BiDirectional Rapidly-exploring Random Tree (cbirrt) plugin contains the cbirrt problem, which parses commands from python or matlab and
 * calls the cbirrt planner. The cbirrt planner is a bidirectional sampling-based rrt which plans
 * on constraint manifolds induced by pose constraints as well as other constraints. This version of the software implements
 * pose constraints as Task Space Region (TSR) Chains. Pose constraints can apply to the start, goal, or entire path.
 * The planner is also set up to work with balance constraints.
 */
#include <openrave/plugin.h>
#include "stdafx.h"

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    cout<<"Interface Create: "<<interfacename<<endl;

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


void ElasticStrips::SetActiveRobots(const std::vector<RobotBasePtr >& robots)
{

    if( robots.size() == 0 ) {
        RAVELOG_WARNA("No robots to plan for\n");
        return;
    }

    vector<RobotBasePtr >::const_iterator itrobot;
    FORIT(itrobot, robots) {
        if( strcmp((*itrobot)->GetName().c_str(), _strRobotName.c_str() ) == 0  ) {
            _esRobot = *itrobot;
            break;
        }
    }

    if( _esRobot == NULL ) {
        RAVELOG_ERRORA("Failed to find %S\n", _strRobotName.c_str());
        return;
    }
}

int ElasticStrips::RunElasticStrips(ostream& sout, istream& sinput)
{
    // Lock environment mutex
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());

    boost::shared_ptr<ESParameters> params(new ESParameters());
    //params.reset(new ESParameters());

    RAVELOG_DEBUG("Starting Elastic Strips...\n");
    RAVELOG_INFO("Checking ravelog_info\n");


    RAVELOG_INFO("Initialize robot object.\n");
    // Initialize _esRobot

    sinput >> _strRobotName;
    RAVELOG_INFO("Robot Name: %s\n",_strRobotName.c_str());

    std::vector<RobotBasePtr> robots;

    GetEnv()->GetRobots(robots);
    SetActiveRobots(robots);

    _numdofs = _esRobot->GetActiveDOF();


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

    // Store the trajectory
    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
    ptraj->Init(_esRobot->GetActiveConfigurationSpecification());

    RAVELOG_INFO("Loading command\n");

    // Command string holds the current command
    string cmd;

    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        // RAVELOG_INFO(cmd);
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
            std::vector<dReal> temp_waypoint(_numdofs);
            int temp_index;
            for(int i = 0; i < num_waypoints; i++)
            {
                sinput >> temp_index;
                for(int j = 0; j < _numdofs; j++)
                {
                    sinput >> temp_q;
                    temp_waypoint[j] = temp_q;
                }
                ptraj->Insert(temp_index,temp_waypoint,_esRobot->GetActiveConfigurationSpecification());
            }
        }
        else if(stricmp(cmd.c_str(), "desiredmanippose") == 0) {
            // Specify the desired pose of each configuration in the trajectory.
            // Index, Number of specified pose in the configuration, The transformation.
            int temp_index;
            int numspecifiedmanip;
            int temp_manip_index;
            sinput >> temp_index;
            sinput >> numspecifiedmanip;

            std::map<string,Transform> temp_manip_pose;
            for(int i = 0; i < numspecifiedmanip; i++)
            {
                sinput >> temp_manip_index;
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

                temp_manip_pose.insert(std::pair<string,Transform>(_esRobot->GetManipulators()[temp_manip_index]->GetName(), temp_tf));
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
                params->_control_points.insert(std::pair<string,Vector>(tempstring, _esRobot->GetLink(tempstring)->GetCOMOffset()));
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
                for(unsigned int j = 0; j < bodies.size(); j++)
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

    RAVELOG_INFO("Commmand loaded\n");

    if(params->balance_mode == BALANCE_SUPPORT_POLYGON)
    {
        RAVELOG_INFO("Balance Mode: Support Polygon.\n");
        if(supportlinks.size() == 0)
        {
            RAVELOG_INFO("ERROR: Must specify support links to do balancing\n");
            return false;
        }

        Balance b(_esRobot, supportlinks, polyscale, polytrans);
        balance_checker = b;

    }
    else if(params->balance_mode == BALANCE_GIWC)
    {
        RAVELOG_INFO("Balance Mode: GIWC.\n");
        Balance b(_esRobot, gravity, support_manips, support_mus);
        balance_checker = b;
    }

    RAVELOG_INFO("Balance checker initilize done.\n");


    unsigned long starttime = timeGetTime();


    RAVELOG_INFO("Initialize planning variables.\n");

    InitPlan(params);

    RAVELOG_INFO("Planning variables initialized. Now start planning.\n");
    
    std::vector<dReal> qResult(_numdofs);
        
    if(PlanPath(ptraj) == PS_HasSolution)
    {

        int timetaken = timeGetTime() - starttime;
        RAVELOG_INFO("Num of waypoints: %i\n",ptraj->GetNumWaypoints());
        for(int w = 0; w < ptraj->GetNumWaypoints(); w++)
        {
            ptraj->GetWaypoint(w,qResult);
            sout<< w << " ";
            for(int i = 0; i < qResult.size(); i++)
            {
                sout << qResult[i] << " ";
            }
        }
        
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

Transform ElasticStrips::ForwardKinematics(std::vector<dReal> qs,string manip_name)
{
    _esRobot->SetActiveDOFValues(qs);
    return _esRobot->GetManipulators()[GetManipIndex.find(manip_name)->second]->GetTransform();
}

void ElasticStrips::FindContactRegions()
{
    for(map<size_t, map<string,Transform> >::iterator dmp_it = _parameters->_desired_manip_pose.begin(); dmp_it != _parameters->_desired_manip_pose.end(); dmp_it++) // all contact regions
    {
        map<string,int> temp_contact_region;
        for(map<string,Transform>::iterator p_it = dmp_it->second.begin(); p_it != dmp_it->second.end(); p_it++) // contact regions in the same waypoint
        {
            bool matched_contact_region_found = false;
            Vector desired_trans = p_it->second.trans;
            string desired_manip_name = p_it->first;
            // cout<<"desired manip name: "<<desired_manip_name<<", desired_trans: ("<<desired_trans.x<<","<<desired_trans.y<<","<<desired_trans.z<<")"<<endl;
            for(std::vector< std::pair<string,Vector> >::iterator ccmt_it = contact_consistent_manip_translation.begin(); ccmt_it != contact_consistent_manip_translation.end(); ccmt_it++)
            {
                string temp_manip_name = ccmt_it->first;
                Vector temp_contact_manip_trans = ccmt_it->second;
                if(temp_manip_name == desired_manip_name)
                {
                    if(fabs(desired_trans.x - temp_contact_manip_trans.x)+
                       fabs(desired_trans.y - temp_contact_manip_trans.y)+
                       fabs(desired_trans.z - temp_contact_manip_trans.z) < 0.001)
                    {
                        // cout<<"manipulator: "<<temp_manip_name<<", desired_trans: ("<<desired_trans.x<<","<<desired_trans.y<<","<<desired_trans.z<<"), temp_contact_manip_trans: ("<<temp_contact_manip_trans.x<<","<<temp_contact_manip_trans.y<<","<<temp_contact_manip_trans.z<<")"<<endl;
                        map<size_t, map<string,Transform> >::iterator prev_dmp_it = dmp_it;
                        prev_dmp_it--;
                        int contact_region_index = ccmt_it - contact_consistent_manip_translation.begin();
                        if(contact_consistent_region.find(prev_dmp_it->first)->second.count(desired_manip_name) == 0)
                        {
                            // cout<<"manipulator "<<desired_manip_name<<" is not in contact in the last waypoint"<<endl;
                            break;
                        }
                        int p_contact_region_index = contact_consistent_region.find(prev_dmp_it->first)->second.find(desired_manip_name)->second;
                        // if the contact region index now is the same as the contact region index in the last waypoint with the same manip, it is clustered to the same contact.
                        // cout<<"contact region index: "<<contact_region_index<<", previous contact region index: "<<p_contact_region_index<<endl;
                        if(contact_region_index == p_contact_region_index)
                        {
                            // cout<<"matched contact region found."<<endl;
                            temp_contact_region.insert(std::pair<string,int>(desired_manip_name, contact_region_index));
                            matched_contact_region_found = true;
                            break;
                        }
                    }
                }
            }

            if(!matched_contact_region_found)
            {
                // cout<<"no matched contact region. add a new entry."<<endl;
                temp_contact_region.insert(std::pair<string,int>(desired_manip_name, contact_consistent_manip_translation.size()));
                contact_consistent_manip_translation.push_back(std::pair<string,Vector>(desired_manip_name,desired_trans));
            }
            // cout<<"contact_consistent_manip_translation size: "<<contact_consistent_manip_translation.size()<<endl;
        }

        contact_consistent_region.insert(std::pair<size_t, std::map<string,int> >(dmp_it->first, temp_contact_region));
    }
}

void ElasticStrips::FindContactConsistentManipTranslation(TrajectoryBasePtr ptraj)
{
    dReal contact_region_radius = 0.05;
    std::vector< std::vector<Vector> > temp_contact_consistent_manip_translations(contact_consistent_manip_translation.size()); 
    for(unsigned int w = 0; w < ptraj->GetNumWaypoints(); w++)
    {
        vector<dReal> qt;
        ptraj->GetWaypoint(w,qt);

        std::map<string,int> contact_regions = contact_consistent_region.find(w)->second;
        
        for(std::map<string,int>::iterator cr_it = contact_regions.begin(); cr_it != contact_regions.end(); cr_it++)
        {
            string temp_manip_name = cr_it->first;
            int temp_region_index = cr_it->second;
            Transform local_desired_pose_frame = _parameters->_desired_manip_pose.find(w)->second.find(temp_manip_name)->second;
            Vector contact_position = ForwardKinematics(qt,temp_manip_name).trans;
            // cout<<"local_desired_pose_frame: "<<local_desired_pose_frame<<endl;
            // cout<<"contact_position: "<<contact_position.x<<" "<<contact_position.y<<" "<<contact_position.z<<endl;
            contact_position = local_desired_pose_frame.inverse() * contact_position;
            // cout<<"contact_position: "<<contact_position.x<<" "<<contact_position.y<<" "<<contact_position.z<<endl;
            contact_position.z = 0;
            dReal dist_to_contact_region_center = sqrt(contact_position.x*contact_position.x + contact_position.y*contact_position.y);
            if(dist_to_contact_region_center > contact_region_radius)
            {
                contact_position.x *= (contact_region_radius/dist_to_contact_region_center);
                contact_position.y *= (contact_region_radius/dist_to_contact_region_center);
            }
            // cout<<"contact_position: "<<contact_position.x<<" "<<contact_position.y<<" "<<contact_position.z<<endl;
            contact_position = local_desired_pose_frame * contact_position;
            // cout<<"contact_position: "<<contact_position.x<<" "<<contact_position.y<<" "<<contact_position.z<<endl;
            temp_contact_consistent_manip_translations.at(temp_region_index).push_back(contact_position);
        }
    }

    for(std::vector< std::vector<Vector> >::iterator tccmt_it = temp_contact_consistent_manip_translations.begin(); tccmt_it != temp_contact_consistent_manip_translations.end(); tccmt_it++)
    {
        Vector v(0,0,0);
        int contact_region_index = tccmt_it - temp_contact_consistent_manip_translations.begin();
        for(std::vector<Vector>::iterator it = tccmt_it->begin(); it != tccmt_it->end(); it++)
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


void ElasticStrips::InitPlan(boost::shared_ptr<ESParameters> params)
{
    //0. parameter initialization
    _esRobot->GetActiveDOFLimits(_lowerLimit,_upperLimit);

    for(int i = 0; i < _esRobot->GetManipulators().size(); i++)
    {
        GetManipIndex.insert(std::pair<string,int>(_esRobot->GetManipulators()[i]->GetName(),i));
    }

    //1. receive command
    // RobotBase::ManipulatorConstPtr activemanip_p = _esRobot->GetActiveManipulator();

    // dReal* qResult = &(*result.get())[0];
    // const dReal* pFreeParameters = &vFreeParameters[0];
    // solutionpath.resize(0);
    // // _targmanips.resize(0);
    // // _targtms.resize(0);
    // movementlimit = INF;

    // bool bsuccess = true;

    _parameters = params;

    //Group the contact regions
    FindContactRegions();

                
    balance_step.ReSize(_numdofs);
    balance_step = 0;
    oa_step.ReSize(_numdofs);
    oa_step = 0;
    zrpy_step.ReSize(_numdofs);
    zrpy_step = 0;
    xy_step.ReSize(_numdofs);
    xy_step = 0;            
    pc_step.ReSize(_numdofs);
    pc_step = 0;
    m_step.ReSize(_numdofs);
    m_step = 0;

    _tasktm.ReSize(3,3);
    _E_rpy.ReSize(3,3);

    _Jp.ReSize(3,_numdofs);
    _Jp0.ReSize(3,_numdofs);

    _Jr.ReSize(3,_numdofs);
    _Jr0.ReSize(3,_numdofs);
    _Jr_proper.ReSize(3,_numdofs);

    step.ReSize(_numdofs);
    step = 0.0;
}

void ElasticStrips::GetRepulsiveVector(Vector& repulsive_vector, std::multimap<string,Vector>::iterator& control_point)
{
    dReal repulse_dist = 1000;
    dReal repulse_constant = -1;
    Transform link_transform = _esRobot->GetLink(control_point->first)->GetTransform();
    Vector control_point_global_position = link_transform*control_point->second;
    repulsive_vector = Vector(0,0,0);
    dReal shortest_dist = 100000000;

    //environment collision
    for(std::vector<KinBodyPtr>::iterator obs_it = _parameters->_esObstacle.begin(); obs_it != _parameters->_esObstacle.end(); obs_it++)
    {
        std::vector<KinBody::LinkPtr> ObstacleLink = (*obs_it)->GetLinks();
        for(std::vector<KinBody::LinkPtr>::iterator link_it = ObstacleLink.begin(); link_it != ObstacleLink.end(); link_it++)
        {
            if(GetEnv()->CheckCollision(_esRobot->GetLink(control_point->first),(*link_it)))
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
    for(std::vector<std::pair<string,string> >::iterator sc_it = _parameters->_self_collision_checking_pairs.begin(); sc_it != _parameters->_self_collision_checking_pairs.end(); sc_it++)
    {
        string link_1 = (*sc_it).first;
        string link_2 = (*sc_it).second;
        if(control_point->first == link_1 || control_point->first == link_2)
        {
            if(GetEnv()->CheckCollision(_esRobot->GetLink(link_1),_esRobot->GetLink(link_2)))
            {
                RaveVector<dReal> repulsive_vector_component(0,0,0);
                string other_link = (control_point->first == link_1) ? link_2 : link_1;
                RaveVector<dReal> other_link_centroid = _esRobot->GetLink(other_link)->GetTransform().trans;
                repulsive_vector_component = (control_point_global_position - other_link_centroid).normalize3();
                repulsive_vector = repulsive_vector + repulsive_vector_component;
                shortest_dist = 0;
            }
        }
    }

    if(repulsive_vector.lengthsqr3() != 0)
    {
        repulsive_vector = repulse_constant * exp(-shortest_dist) * repulsive_vector * (1/repulsive_vector.lengthsqr3());
        // cout<<"Link: "<<control_point->first<<", Repulsive Vector: ("<<repulsive_vector.x<<","<<repulsive_vector.y<<","<<repulsive_vector.z<<")"<<endl;
    }

    //for each obstacle, find its geometry type
    //calculate distance between the control point and the obstacle
    //calculate the repulsive from each obstacle according to ther relative position and distance
    //sum the repulsive vector
}

void ElasticStrips::RemoveBadJointJacobianCols(NEWMAT::Matrix& J, size_t w)
{
    if(!badjointinds.at(w).empty())
    {
        for(int j = 0; j < badjointinds.at(w).size(); j++)
            for(int k = 1; k <= J.Nrows(); k++)
                J(k,badjointinds.at(w).at(j)+1) = 0;     
    }
}


void ElasticStrips::UpdateZRPYandXYJacobianandStep(Transform taskframe_in, size_t w)
{
    map<string,Transform> desired_manip_pose = _parameters->_desired_manip_pose.find(w)->second;

    // XY
    JXY.ReSize(2*desired_manip_pose.size(),_numdofs);
    JXYplus.ReSize(_numdofs,2*desired_manip_pose.size());
    Mxy.ReSize(2*desired_manip_pose.size());
    Mxyinv.ReSize(2*desired_manip_pose.size());
    dxy.ReSize(2*desired_manip_pose.size());
    Regxy.ReSize(2*desired_manip_pose.size());
    Regxy = 0.0001;

    // Z
    JZ.ReSize(1,_numdofs);
    JRPY.ReSize(3,_numdofs);
    
    JZRPY.ReSize(4*desired_manip_pose.size(),_numdofs);
    JZRPYplus.ReSize(_numdofs,4*desired_manip_pose.size());
    Mzrpy.ReSize(4*desired_manip_pose.size());
    Mzrpyinv.ReSize(4*desired_manip_pose.size());
    dzrpy.ReSize(4*desired_manip_pose.size());
    Regzrpy.ReSize(4*desired_manip_pose.size());
    Regzrpy = 0.0001;

    cogtarg = Vector(0,0,0);

    int i = 0;
    vector<string> support_links;
    for(map<string,Transform>::iterator dmp_it = desired_manip_pose.begin(); dmp_it != desired_manip_pose.end(); dmp_it++, i++)
    {
        string manip_name = dmp_it->first;

        int region_index = contact_consistent_region.find(w)->second.find(manip_name)->second;
        Vector contact_consistent_point = contact_consistent_manip_translation.at(region_index).second;

        Transform contact_consistent_point_frame = dmp_it->second;
        contact_consistent_point_frame.trans.x = contact_consistent_point.x;
        contact_consistent_point_frame.trans.y = contact_consistent_point.y;
        contact_consistent_point_frame.trans.z = contact_consistent_point.z;

        // Jacobian
        RobotBase::ManipulatorPtr target_manip = _esRobot->GetManipulators()[GetManipIndex.find(manip_name)->second];
        std::vector<dReal> temp;

        _esRobot->CalculateActiveJacobian(target_manip->GetEndEffector()->GetIndex(), target_manip->GetTransform().trans, temp);
        memcpy(_Jp0.Store(),&temp[0],temp.size()*sizeof(dReal));

        _esRobot->CalculateActiveAngularVelocityJacobian(target_manip->GetEndEffector()->GetIndex(), temp);
        memcpy(_Jr0.Store(),&temp[0],temp.size()*sizeof(dReal));

        _TMtask = contact_consistent_point_frame.inverse() * taskframe_in.inverse();
        _tasktm(1,1) = _TMtask.m[0];        _tasktm(1,2) = _TMtask.m[1];        _tasktm(1,3) = _TMtask.m[2];
        _tasktm(2,1) = _TMtask.m[4];        _tasktm(2,2) = _TMtask.m[5];        _tasktm(2,3) = _TMtask.m[6];
        _tasktm(3,1) = _TMtask.m[8];        _tasktm(3,2) = _TMtask.m[9];        _tasktm(3,3) = _TMtask.m[10];

        _Jp = _tasktm * _Jp0;
        _Jr = _tasktm * (-_Jr0);

        JXY.Rows(i*2+1,i*2+2) = _Jp.Rows(1,2);
        JZ.Row(1) = _Jp.Row(3);

        //convert current rotation to euler angles (RPY) 
        QuatToRPY(contact_consistent_point_frame.inverse()*taskframe_in.inverse()*target_manip->GetTransform(),_psi,_theta,_phi);
        //RAVELOG_INFO("psi:  %f  theta:  %f   phi:  %f\n",psi,theta,phi);

        Cphi = cos(_phi);
        Ctheta = cos(_theta);
        Cpsi = cos(_psi);

        Sphi = sin(_phi);
        Stheta = sin(_theta);
        Spsi = sin(_psi);

        _E_rpy(1,1) = Cphi/Ctheta;         _E_rpy(1,2) = Sphi/Ctheta;         _E_rpy(1,3) = 0;
        _E_rpy(2,1) = -Sphi;               _E_rpy(2,2) = Cphi;                _E_rpy(2,3) = 0;
        _E_rpy(3,1) = Cphi*Stheta/Ctheta;  _E_rpy(3,2) = Sphi*Stheta/Ctheta;  _E_rpy(3,3) = 1;

        _Jr_proper = _E_rpy * _Jr;

        JRPY = -_Jr_proper;

        JZRPY.Row(i*4+1) = JZ.Row(1);
        JZRPY.Rows(i*4+2,i*4+4) = JRPY.Rows(1,3);

        // Step
        NEWMAT::ColumnVector dxyzrpy;
        dxyzrpy.ReSize(6);

        TransformDifference(dxyzrpy.Store(), contact_consistent_point_frame, target_manip->GetTransform());


        //velocity, can be written as force
        dxy(i*2+1) = dxyzrpy(1);
        dxy(i*2+2) = dxyzrpy(2);
        dzrpy(i*4+1) = dxyzrpy(3);
        dzrpy(i*4+2) = dxyzrpy(4);
        dzrpy(i*4+3) = dxyzrpy(5);
        dzrpy(i*4+4) = dxyzrpy(6);

        xy_error = xy_error + (dxyzrpy(1) * dxyzrpy(1)) + (dxyzrpy(2) * dxyzrpy(2));
        z_error = z_error + (dxyzrpy(3) * dxyzrpy(3));
        rpy_error = rpy_error + (dxyzrpy(4) * dxyzrpy(4)) + (dxyzrpy(5) * dxyzrpy(5)) + (dxyzrpy(6) * dxyzrpy(6));;

        //dirty code to decide cogtarg:(specialize to escher robot)
        if(manip_name == "l_leg")
        {
            cogtarg.x += contact_consistent_point.x;
            cogtarg.y += contact_consistent_point.y;
            support_links.push_back("l_foot");
        }
        else if(manip_name == "r_leg")
        {
            cogtarg.x += contact_consistent_point.x;
            cogtarg.y += contact_consistent_point.y;
            support_links.push_back("r_foot");
        }
        //************//
        // getchar();
    }

    RemoveBadJointJacobianCols(JXY,w);
    Mxy << JXY*JXY.t() + Regxy;
    invConditioningBound(10000,Mxy,Mxyinv);
    JXYplus = JXY.t()*Mxyinv;

    RemoveBadJointJacobianCols(JZRPY,w);
    Mzrpy << JZRPY*JZRPY.t() + Regzrpy;
    invConditioningBound(10000,Mzrpy,Mzrpyinv);
    JZRPYplus = JZRPY.t()*Mzrpyinv;

    cogtarg.x /= support_links.size();
    cogtarg.y /= support_links.size();
    // if(support_links.size() == 1) // standing with one foot
    // {
    //     Vector l_foot_transform = _esRobot->GetLink("l_foot")->GetTransform().trans;
    //     Vector r_foot_transform = _esRobot->GetLink("r_foot")->GetTransform().trans;
    //     cogtarg.x = (l_foot_transform.x + r_foot_transform.x)/2.0;
    // }


    if(_parameters->balance_mode == BALANCE_SUPPORT_POLYGON)
    {
        vector<dReal> qs;
        _esRobot->GetActiveDOFValues(qs);
        balance_checker.RefreshBalanceParameters(qs,support_links);
    }
    else if(_parameters->balance_mode == BALANCE_GIWC)
    {
        // TODO: add a refreshblanaceparameter function for GIWC
    }


    xy_error = sqrt(xy_error);
    z_error = sqrt(z_error);
    rpy_error = sqrt(rpy_error);

    dxy = cxy * dxy;
    dzrpy = czrpy * dzrpy;
}


void ElasticStrips::UpdateCOGJacobianandStep(Transform taskframe_in,size_t w)
{
    JCOG = 0.0;

    curcog = Vector(0,0,0,0);
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
        memcpy(_Jp0.Store(),&temp[0],temp.size()*sizeof(dReal));

        _Jp = _tasktm * _Jp0;

        JCOG = JCOG + ((*itlink)->GetMass()*(_Jp.Rows(1,2)));

        curcog += ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset() * (*itlink)->GetMass());
        fTotalMass += (*itlink)->GetMass();
    }

    if( fTotalMass > 0 )
        curcog /= fTotalMass;
   
    JCOG = JCOG/fTotalMass;
    RemoveBadJointJacobianCols(JCOG,w);
    Mcog << JCOG*JCOG.t() + Regcog;
    invConditioningBound(10000,Mcog,Mcoginv);
    JCOGplus = JCOG.t()*Mcoginv;

    if(!balance_checker.CheckSupport(curcog))
    {
        dcog(1) = (curcog.x - cogtarg.x);
        dcog(2) = (curcog.y - cogtarg.y);
    }
    else
    {
        dcog(1) = 0;
        dcog(2) = 0;
    }

    dcog = ccog * dcog;

}

void ElasticStrips::UpdateOAJacobianandStep(Transform taskframe_in, size_t w)
{
    //calculate the velocity of each control point
    //calculate jacobian for each control point to generate the joint angular velocity
    //sum the joint angular velocity

    std::map<string,Vector> control_points_in_collision;
    std::map<string,Vector> control_point_repulsive_vector;
    std::map<string,NEWMAT::Matrix> control_point_jacobian;

    for(std::map<string,Vector>::iterator ctrl_it = _parameters->_control_points.begin(); ctrl_it != _parameters->_control_points.end(); ctrl_it++)
    {
        Vector repulsive_vector;
        GetRepulsiveVector(repulsive_vector, ctrl_it);

        if(repulsive_vector.lengthsqr3() != 0)
        {
            _Jp.ReSize(3,_numdofs);

            _TMtask = TransformMatrix(taskframe_in.inverse());
            _tasktm(1,1) = _TMtask.m[0];        _tasktm(1,2) = _TMtask.m[1];        _tasktm(1,3) = _TMtask.m[2];
            _tasktm(2,1) = _TMtask.m[4];        _tasktm(2,2) = _TMtask.m[5];        _tasktm(2,3) = _TMtask.m[6];
            _tasktm(3,1) = _TMtask.m[8];        _tasktm(3,2) = _TMtask.m[9];        _tasktm(3,3) = _TMtask.m[10];

            KinBody::LinkPtr target_link = _esRobot->GetLink(ctrl_it->first);
            std::vector<dReal> temp;

            _esRobot->CalculateActiveJacobian(target_link->GetIndex(), (target_link->GetTransform() * ctrl_it->second), temp);
            memcpy(_Jp0.Store(),&temp[0],temp.size()*sizeof(dReal));

            _Jp = _tasktm * _Jp0;

            control_points_in_collision.insert(*ctrl_it);
            control_point_repulsive_vector.insert(std::pair<string,Vector>(ctrl_it->first,repulsive_vector));
            control_point_jacobian.insert(std::pair<string,NEWMAT::Matrix>(ctrl_it->first,_Jp));
        }
    }

    if(!control_points_in_collision.empty())
    {
        NEWMAT::Matrix Jtemp;
        Jtemp.ReSize(0,_numdofs);
        doa.ReSize(3*control_points_in_collision.size());

        int point_index = 0;

        //stack the repulsive vector and jacobian matrix
        for(std::map<string,Vector>::iterator ctrl_it = control_points_in_collision.begin(); ctrl_it != control_points_in_collision.end(); ctrl_it++)
        {
            Jtemp &= control_point_jacobian.find(ctrl_it->first)->second;
            doa(point_index*3+1) = control_point_repulsive_vector.find(ctrl_it->first)->second.x;
            doa(point_index*3+2) = control_point_repulsive_vector.find(ctrl_it->first)->second.y;
            doa(point_index*3+3) = control_point_repulsive_vector.find(ctrl_it->first)->second.z;
            point_index++;
        }

        JOA = Jtemp;

        RemoveBadJointJacobianCols(JOA,w);
        Regoa.ReSize(JOA.Nrows());
        Regoa = 0.0001;
        Moa.ReSize(JOA.Nrows());
        Moainv.ReSize(JOA.Nrows());
        Moa << (JOA*JOA.t()) + Regoa;
        invConditioningBound(10000,Moa,Moainv);
        JOAplus.ReSize(_numdofs,3*control_points_in_collision.size());
        JOAplus = JOA.t()*Moainv;

        doa = coa * doa / control_points_in_collision.size();

        bInCollision = true;
    }
    else
    {
        doa.ReSize(1);
        JOA.ReSize(1,_numdofs);
        JOAplus.ReSize(_numdofs,1);
        doa = 0.0;
        JOA = 0.0;
        JOAplus = 0.0;
        bInCollision = false;
    }

}



void ElasticStrips::UpdatePCJacobianandStep(Transform taskframe_in,size_t w)
{
    NEWMAT::Matrix Jtemp;
    Jtemp.ReSize(0,_numdofs);
    int i = 0;
    for(map<string,Transform>::iterator pc_it = _parameters->_posture_control.begin(); pc_it != _parameters->_posture_control.end(); pc_it++, i++)
    {
        KinBody::LinkPtr target_link = _esRobot->GetLink(pc_it->first);

        // Jacobian
        std::vector<dReal> temp;

        _esRobot->CalculateActiveJacobian(target_link->GetIndex(), target_link->GetTransform().trans, temp);
        memcpy(_Jp0.Store(),&temp[0],temp.size()*sizeof(dReal));

        _esRobot->CalculateActiveAngularVelocityJacobian(target_link->GetIndex(), temp);
        memcpy(_Jr0.Store(),&temp[0],temp.size()*sizeof(dReal));


        _TMtask = TransformMatrix(taskframe_in.inverse());
        _tasktm(1,1) = _TMtask.m[0];        _tasktm(1,2) = _TMtask.m[1];        _tasktm(1,3) = _TMtask.m[2];
        _tasktm(2,1) = _TMtask.m[4];        _tasktm(2,2) = _TMtask.m[5];        _tasktm(2,3) = _TMtask.m[6];
        _tasktm(3,1) = _TMtask.m[8];        _tasktm(3,2) = _TMtask.m[9];        _tasktm(3,3) = _TMtask.m[10];

        _Jp = _tasktm * _Jp0;

        _Jr = _tasktm * (-_Jr0);

        //convert current rotation to euler angles (RPY) 
        QuatToRPY(taskframe_in.inverse()*target_link->GetTransform(),_psi,_theta,_phi);
        //RAVELOG_INFO("psi:  %f  theta:  %f   phi:  %f\n",psi,theta,phi);

        Cphi = cos(_phi);
        Ctheta = cos(_theta);
        Cpsi = cos(_psi);

        Sphi = sin(_phi);
        Stheta = sin(_theta);
        Spsi = sin(_psi);

        _E_rpy(1,1) = Cphi/Ctheta;         _E_rpy(1,2) = Sphi/Ctheta;         _E_rpy(1,3) = 0;
        _E_rpy(2,1) = -Sphi;               _E_rpy(2,2) = Cphi;                _E_rpy(2,3) = 0;
        _E_rpy(3,1) = Cphi*Stheta/Ctheta;  _E_rpy(3,2) = Sphi*Stheta/Ctheta;  _E_rpy(3,3) = 1;

        _Jr_proper = _E_rpy * _Jr;

        Jtemp &= _Jp;
        Jtemp &= -_Jr_proper;


        // Step
        Transform desired_link_transform = taskframe_in.inverse() * pc_it->second * _esRobot->GetLink("torso")->GetTransform();
        TransformDifference(dpc.Store() + i*6, desired_link_transform, taskframe_in.inverse()*target_link->GetTransform());
    }

    JPC = Jtemp;
    RemoveBadJointJacobianCols(JPC,w);
    Mpc << (JPC*JPC.t()) + Regpc;
    invConditioningBound(10000,Mpc,Mpcinv);
    JPCplus = JPC.t()*Mpcinv;

    dpc = cpc * dpc;

}


dReal ElasticStrips::TransformDifference(dReal * dx,Transform tm_ref, Transform tm_targ)
{
    _tmtemp = tm_ref.inverse()*tm_targ;

    dx[0] = _tmtemp.trans.x;
    dx[1] = _tmtemp.trans.y;
    dx[2] = _tmtemp.trans.z;

    QuatToRPY(_tmtemp,dx[3],dx[4],dx[5]);

    _sumsqr = 0;
    for(int i = 0; i < 6; i++)
        _sumsqr += dx[i]*dx[i];

    return sqrt(_sumsqr);
}

OpenRAVE::PlannerStatus ElasticStrips::PlanPath(TrajectoryBasePtr ptraj)
{
    OpenRAVE::PlannerStatus result = PS_HasSolution;
    dReal maxstep, magnitude;

    RAVELOG_INFO("Initialize posture control and balance variables.\n");

    // Initialize invaraint Jacobian dimension of each constriant.
    if(_parameters->bPOSTURE_CONTROL)
    {
        JPC.ReSize(6*_parameters->_posture_control.size(),_numdofs);
        JPCplus.ReSize(_numdofs,6*_parameters->_posture_control.size());
        Mpc.ReSize(6*_parameters->_posture_control.size());
        Mpcinv.ReSize(6*_parameters->_posture_control.size());
        Regpc.ReSize(6*_parameters->_posture_control.size());
        Regpc = 0.0001;
        dpc.ReSize(6*_parameters->_posture_control.size());
    }

    if(_parameters->balance_mode != BALANCE_NONE)
    {
        JCOG.ReSize(2,_numdofs);
        JCOGplus.ReSize(_numdofs,2);
        Mcog.ReSize(2);
        Mcoginv.ReSize(2);
        Regcog.ReSize(2);
        Regcog = 0.0001;
        dcog.ReSize(2);
    }
    
    RAVELOG_INFO("Initialize waypoints status tracker.\n");

    for(unsigned int w = 0; w < ptraj->GetNumWaypoints(); w++)
    {
        stable_waypoint.insert(std::pair<size_t,bool>(w,false));
    }

    // std::pair<dReal,dReal> prismatic_x_limit;
    // // prismatic_x_limit.first = 


    TrajectoryBasePtr ftraj = RaveCreateTrajectory(GetEnv(),"");
    ftraj->Init(_esRobot->GetActiveConfigurationSpecification());

    ftraj->Clone(ptraj,0);

    bool all_waypoints_stable;

    RAVELOG_INFO("Elastic Strips main loop starts.\n");

    std::vector<size_t> once_stable_waypoint;

    badjointinds.resize(ftraj->GetNumWaypoints());
           
    for(int k = 0; k < 200; k++) // modify the configuration
    {
        RAVELOG_INFO("Iteration: %i\n",k);
        TrajectoryBasePtr ntraj = RaveCreateTrajectory(GetEnv(),"");
        ntraj->Init(_esRobot->GetActiveConfigurationSpecification());
        // ntraj->Remove(0,ntraj->GetNumWaypoints());
        // RAVELOG_INFO("ntraj waypoint number: %i\n",ntraj->GetNumWaypoints());
        all_waypoints_stable = true;

        RAVELOG_INFO("Find the contact consistent manipulator transform.\n");
        // Calculate the contact consistent manipulator translation
        FindContactConsistentManipTranslation(ftraj);

        once_stable_waypoint.clear();

        // for(unsigned int w = 0; w < ftraj->GetNumWaypoints(); w++)
        // {
        //     std::vector<dReal> qs(_numdofs);
        //     ftraj->GetWaypoint(w,qs);
        //     _esRobot->SetActiveDOFValues(qs);
        //     cout<<"Waypoint "<<w<<":"<<endl;
        //     cout<<"left_foot: "<<_esRobot->GetLink("l_foot")->GetTransform().trans.x<<", "<<_esRobot->GetLink("l_foot")->GetTransform().trans.y<<endl;
        //     cout<<"right_foot: "<<_esRobot->GetLink("r_foot")->GetTransform().trans.x<<", "<<_esRobot->GetLink("r_foot")->GetTransform().trans.y<<endl;
        // }

        // getchar();

        for(unsigned int w = 0; w < ftraj->GetNumWaypoints(); w++) // for each configuration in the trjectory
        {
            RAVELOG_INFO("Waypoint: %i\n",w);

            // Get the initial configuration
            std::vector<dReal> qs(_numdofs); // the initial configuration
            ftraj->GetWaypoint(w,qs);
            _esRobot->SetActiveDOFValues(qs);
            GetEnv()->UpdatePublishedBodies();
            std::vector<dReal> qs_old = qs; // store the configuration before taking step.
            if(_parameters->balance_mode != BALANCE_NONE)
                balance_checker.RefreshBalanceParameters(qs);

            if(stable_waypoint.find(w)->second == true)
            {
                // ntraj->Insert(w,qs,_esRobot->GetActiveConfigurationSpecification());
                stable_waypoint.find(w)->second = false;
                // continue;
            }
            else
            {
                RAVELOG_INFO("Waypoint %i is not stable yet.\n",w);
                all_waypoints_stable = false;
            }

            bLimit = false;
            // badjointinds.clear();

            bInCollision = false;

            maxstep = 0.1 * contact_consistent_region.find(w)->second.size();

            RAVELOG_INFO("Move configuration according to the specified constraints.\n");

            do
            {
                // Calculate each Jacobian
                qs_old = qs;
                xy_error = 0;
                z_error = 0;
                rpy_error = 0;

                // 1. Z
                // 2. XY
                UpdateZRPYandXYJacobianandStep(Transform(),w);

                // 3. Balance / COG / Update COG
                if(_parameters->balance_mode != BALANCE_NONE)
                {
                    UpdateCOGJacobianandStep(Transform(),w);
                }

                // 4. Obstacle Avoidance
                if(_parameters->bOBSTACLE_AVOIDANCE)
                {
                    UpdateOAJacobianandStep(Transform(),w);
                }


                //may require another condition to check posture control
                if((xy_error < epsilon) &&
                   (_parameters->balance_mode == BALANCE_NONE || balance_checker.CheckSupport(curcog)) &&
                   (_parameters->bOBSTACLE_AVOIDANCE == false || bInCollision == false))
                {
                    stable_waypoint.find(w)->second = true;
                    RAVELOG_INFO("Waypoint %i is stable.\n",w);
                    once_stable_waypoint.push_back(w);
                    break;
                }
                else
                {
                    cout<<"Reach Point: "<<(xy_error < epsilon)<<endl;
                    cout<<"balanced: "<<(_parameters->balance_mode == BALANCE_NONE || balance_checker.CheckSupport(curcog))<<endl;
                    cout<<"free of collision: "<<(_parameters->bOBSTACLE_AVOIDANCE == false || bInCollision == false)<<endl;
                }
                
                // 5. Posture Control (relative to robot base)
                if(_parameters->bPOSTURE_CONTROL)
                {
                    UpdatePCJacobianandStep(Transform(),w);
                }


                // Calculate steps
                if(xy_error > maxstep)
                    magnitude = maxstep/xy_error;
                else
                    magnitude = 1;

                // NEWMAT::ColumnVector tempstep;
                // tempstep.ReSize(_numdofs);
                step = 0;
                balance_step = 0;
                oa_step = 0;
                zrpy_step = 0;
                xy_step = 0; 
                pc_step = 0;
                m_step = 0;

                string highest_priority = "None";

                if(_parameters->bPOSTURE_CONTROL)
                {
                    pc_step = JPCplus * dpc;
                    highest_priority = "PC";
                    JHP = &JPC;
                    JHPplus = &JPCplus;
                    dhp = &dpc;
                }
                else
                {
                    pc_step = 0;
                }

                if(_parameters->balance_mode != BALANCE_NONE)
                {
                    balance_step = JCOGplus*dcog + (NEWMAT::IdentityMatrix(_numdofs) - JCOGplus*JCOG) * pc_step;
                    highest_priority = "BALANCE";
                    JHP = &JCOG;
                    JHPplus = &JCOGplus;
                    dhp = &dcog;
                }
                else
                {
                    balance_step = pc_step;
                }

                if(_parameters->bOBSTACLE_AVOIDANCE && bInCollision)
                {
                    highest_priority = "OA";
                    JHP = &JOA;
                    JHPplus = &JOAplus;
                    dhp = &doa;
                }

                if(highest_priority == "None")
                {
                    JM = JXY;
                    JMplus = JXYplus;
                    dm = dxy;
                }
                else
                {
                    JM.ReSize(0,_numdofs);
                    JM &= JXY;
                    JM &= (*JHP);

                    dm.ReSize(0);
                    dm &= dxy;
                    dm &= (*dhp);

                    JMplus.ReSize(_numdofs,JM.Nrows());
                    Mm.ReSize(JM.Nrows());
                    Mminv.ReSize(JM.Nrows());
                    Regm.ReSize(JM.Nrows());
                    Regm = 0.0001;

                    Mm << (JM*JM.t()) + Regm;
                    invConditioningBound(10000,Mm,Mminv);
                    JMplus = JM.t()*Mminv;                  
                }

                if(highest_priority == "None" or highest_priority == "PC")
                {
                    m_step = JMplus*dm;                    
                }
                else if(highest_priority == "BALANCE")
                {
                    m_step = JMplus*dm + (NEWMAT::IdentityMatrix(_numdofs) - JMplus*JM) * pc_step;
                }
                else if(highest_priority == "OA")
                {
                    m_step = JMplus*dm + (NEWMAT::IdentityMatrix(_numdofs) - JMplus*JM) * balance_step;
                }

                step = JZRPYplus * dzrpy + (NEWMAT::IdentityMatrix(_numdofs) - JZRPYplus*JZRPY)*m_step;
                // step = JZRPYplus * dzrpy + (NEWMAT::IdentityMatrix(_numdofs) - JZRPYplus*JZRPY)*(JXYplus*dxy);
                // step = JZRPYplus * dzrpy + (NEWMAT::IdentityMatrix(_numdofs) - JZRPYplus*JZRPY)*(JXYplus*dxy + (NEWMAT::IdentityMatrix(_numdofs) - JXYplus*JXY)*(JCOGplus*dcog));

                step = magnitude * step;

                //add step and check for joint limits
                bLimit = false;
                for(int i = 0; i < _numdofs; i++)
                {
                    qs[i] = qs_old[i] - step(i+1);
                    if(qs[i] < _lowerLimit[i] || qs[i] > _upperLimit[i])
                    {
                        // RAVELOG_INFO("Link: %s, value: %f, step: %f, limit: %f <=> %f\n",_esRobot->GetJointFromDOFIndex(_esRobot->GetActiveDOFIndices()[i])->GetName().c_str(),qs_old[i],step(i+1),_lowerLimit[i],_upperLimit[i]);
                        if(qs[i] < _lowerLimit[i])
                            qs[i] = _lowerLimit[i];
                        if(qs[i] > _upperLimit[i])
                            qs[i] = _upperLimit[i];

                        badjointinds.at(w).push_back(i); //note this will never add the same joint twice, even if bClearBadJoints = false

                        bLimit = true;

                    }
                }

                if(bLimit)
                {
                    qs = qs_old;
                }

                // getchar();

            }while(bLimit);

            RAVELOG_INFO("Move a step.\n");

            // for(int i = 0; i < qs.size(); i++)
            // {
            //     cout<<qs[i]<<' ';
            // }
            // cout<<endl;

            _esRobot->SetActiveDOFValues(qs);
            GetEnv()->UpdatePublishedBodies();

            ntraj->Insert(w,qs,_esRobot->GetActiveConfigurationSpecification());

            // getchar();
        }

        // cout<<"Number of contact consistent positions: "<<contact_consistent_manip_translation.size()<<endl;
        // for(std::vector< std::pair<string,Vector> >::iterator ccmt_it = contact_consistent_manip_translation.begin(); ccmt_it != contact_consistent_manip_translation.end(); ccmt_it++)
        // {
        //     cout<<"contact "<<ccmt_it-contact_consistent_manip_translation.begin()<<", "<<ccmt_it->first<<": "<<ccmt_it->second.x<<", "<<ccmt_it->second.y<<", "<<ccmt_it->second.z<<endl;
        // }

        cout<<"Total Waypoints: "<<ftraj->GetNumWaypoints()<<", Once stable waypoints: ";
        for(vector<size_t>::iterator osw_it = once_stable_waypoint.begin(); osw_it != once_stable_waypoint.end(); osw_it++)
        {
            cout<<*osw_it<<' ';
        }
        cout<<endl;

        // for(unsigned int w = 0; w < ftraj->GetNumWaypoints(); w++)
        // {
        //     std::vector<dReal> qs(_numdofs);
        //     ftraj->GetWaypoint(w,qs);
        //     _esRobot->SetActiveDOFValues(qs);
        //     cout<<"After iteration: "<<k<<", Waypoint "<<w<<":"<<endl;
        //     cout<<"left_foot: "<<_esRobot->GetLink("l_foot")->GetTransform().trans.x<<", "<<_esRobot->GetLink("l_foot")->GetTransform().trans.y<<endl;
        //     cout<<"right_foot: "<<_esRobot->GetLink("r_foot")->GetTransform().trans.x<<", "<<_esRobot->GetLink("r_foot")->GetTransform().trans.y<<endl;
        // }

        // getchar();

        ftraj->Clone(ntraj,0);

        if(all_waypoints_stable)
        {
            ptraj->Clone(ftraj,0);
            RAVELOG_INFO("Iteration used: %i\n",k);
            return result;
        }

    }

    ptraj->Clone(ftraj,0);

    result = PS_Failed;

    return result;
}

Vector ElasticStrips::RPYIdentityOffsets[8] = { Vector(M_PI,M_PI,M_PI),
                                            Vector(M_PI,M_PI,-M_PI),
                                            Vector(M_PI,-M_PI,M_PI),
                                            Vector(M_PI,-M_PI,-M_PI),
                                            Vector(-M_PI,M_PI,M_PI),
                                            Vector(-M_PI,M_PI,-M_PI),
                                            Vector(-M_PI,-M_PI,M_PI),
                                            Vector(-M_PI,-M_PI,-M_PI)};

void ElasticStrips::QuatToRPY(Transform tm, dReal& psi, dReal& theta, dReal& phi)
{
    
    a = tm.rot.x;
    b = tm.rot.y;
    c = tm.rot.z;
    d = tm.rot.w;

    //psi theta and phi will always be between -pi and pi
    psi = atan2(2*a*b + 2*c*d, a*a - b*b - c*c + d*d); //psi
    theta = -asin(2*b*d-2*a*c); //theta
    phi = atan2(2*a*d+2*b*c, a*a + b*b - c*c - d*d); //phi
    

    //go through all the identities and find which one minimizes the total rotational distance
    //don't need to consider +/-2pi b/c all three angles are between -pi and pi
   _min_dist = 10000;
    for(int i =0; i < 9; i++)
    {

        if(i == 0) //for first element, use original values
        {
            _temp_vec.x = psi;
            _temp_vec.y = theta;
            _temp_vec.z = phi;
        }
        else
        {
            _temp_vec.x = psi + RPYIdentityOffsets[i-1].x;
            _temp_vec.y = -theta + RPYIdentityOffsets[i-1].y;//note that theta is negative
            _temp_vec.z = phi + RPYIdentityOffsets[i-1].z;
        }
        
        _temp_dist = _temp_vec.lengthsqr3();
        if(_temp_dist < _min_dist)
        {
            _min_dist = _temp_dist;
            psi = _temp_vec.x;
            theta = _temp_vec.y;
            phi = _temp_vec.z;
        }
    }


    //RAVELOG_INFO("psi: %f, theta: %f, phi: %f\n",psi,theta,phi);
}

int ElasticStrips::invConditioningBound(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed)
{
    int didfix = 0;
    NEWMAT::EigenValues(A,_S,_V);
    // Find the maximum eigenvalue
    dReal maxEig = 0;
    dReal minEig = 0;
    for (int i = 1; i <= _S.Nrows(); ++i){
        dReal e = _S(i);
        if (e > maxEig) maxEig = e;
        if (i == 1 || e < minEig) minEig = e;
    }
    //RAVELOG_INFO("min/max eigenvalue: %f/%f\n", minEig, maxEig);

    dReal minEigDesired = maxEig/maxConditionNumber;
    int notfixcount = 0;
    for (int i = 1; i <= _S.Nrows(); ++i){
        dReal e = _S(i);
        if (e < minEigDesired) {e = minEigDesired; didfix = 1;}
        else
            notfixcount++;
            
        if (maxEig > 100) { e = e/maxEig*100;}
        _S(i) = e;
    }
        

    //this will do the inversion 
    Afixed << _V * _S.i() * _V.t();
    return didfix;
}

void ElasticStrips::PrintMatrix(dReal* pMatrix, int numrows, int numcols, const char * statement)
{
    //return;

    stringstream s;
    s.setf(ios::fixed,ios::floatfield);
    s.precision(5);
    s << "\n"; 
    if(statement != NULL)
        s << statement <<"\n";
    for(int i = 0; i < numrows; i++)
    {
        for(int j = 0; j < numcols; j++)
        {
            s << pMatrix[i*numcols + j] << " \t";
            if(fabs(pMatrix[i*numcols + j]) > 10000)
                RAVELOG_INFO("WARNING: %d %d is huge!!!\n",i,j);
        }
        s << endl;
    }
    RAVELOG_INFO(s.str().c_str());

}