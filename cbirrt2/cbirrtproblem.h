/* Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
   Author: Dmitry Berenson <dberenso@cs.cmu.edu>

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
/** \file cbirrtproblem.h
    \brief Defines the cbirrt problem class.
 */
#ifndef CPROBLEM_H
#define CPROBLEM_H

#define GMPRATIONAL 1
#include <cdd/setoper.h>
#include <cdd/cdd.h>


/// Parses input from python and matlab and calls cbirrtplanner, also contains some other useful openrave functions.
class CBirrtProblem : public ProblemInstance
{
public:
    CBirrtProblem(EnvironmentBasePtr penv);
    virtual ~CBirrtProblem();
    virtual void Destroy();
    virtual int main(const std::string& args);
    virtual void SetActiveRobots(const std::vector<RobotBasePtr>& robots);

    virtual bool SendCommand(std::ostream& sout, std::istream& sinput);

    class Point2D
    {
        public:
            double x;
            double y;
    };

private:

    /// grab a body with the active manipulator
    bool GrabBody(ostream& sout, istream& sinput);

    /// call the generalik solver
    bool DoGeneralIK(ostream& sout, istream& sinput);

    /// check for balance
    bool CheckSupport(ostream& sout, istream& sinput);

    /// call the cbirrt planner
    int RunCBirrt(ostream& sout, istream& sinput);

    /// call the cbirrt planner
    int RunElasticStrips(ostream& sout, istream& sinput);

    /// check if robot is in self-collision
    int CheckSelfCollision(ostream& sout, istream& sinput);

    /// get a joint axis direction
    bool GetJointAxis(ostream& sout, istream& sinput);

    /// get a joint transform
    bool GetJointTransform(ostream& sout, istream& sinput);

    /// execute a trajectory
    bool Traj(ostream& sout, istream& sinput);

    /// function for computing a convex hull using qhull
    void compute_convex_hull(void);

    /// function for computing a 2D convex hull
    int convexHull2D(coordT* pointsIn, int numPointsIn, coordT** pointsOut, int* numPointsOut);

    /// function for computing a 6D convex hull
    int convexHull6D(coordT* pointsIn, int numPointsIn, std::vector< std::vector<double> >& facet_coefficients);

    ///function to compute centroid of 2D polygon
    Point2D compute2DPolygonCentroid(const Point2D* vertices, int vertexCount);

    /// TODO: Document
    void GetSupportPointsForLink(RobotBase::LinkPtr p_link, OpenRAVE::Vector tool_dir, Transform result_tf, std::vector<Vector>& contacts);
    std::vector<Vector> GetSupportPoints(RobotBase::ManipulatorPtr p_manip);

    void GetFrictionCone(OpenRAVE::Vector &center, OpenRAVE::Vector &direction, dReal mu, NEWMAT::Matrix *mat, int offset_r, int offset_c, Transform temp_tf);
    void GetASurf(RobotBase::ManipulatorPtr p_manip, Transform cone_tf, NEWMAT::Matrix *mat, int offset_r);
    void GetAStance(Transform cone_tf, NEWMAT::Matrix* mat, int offset_r);

    /// compute the surface support cone for the given manipulator
    NEWMAT::ReturnMatrix GetSurfaceCone(string& manipname, dReal mu);

    /// compute the GIWC for giwc stability
    NEWMAT::ReturnMatrix GetGIWCSpanForm(std::vector<std::string>& manip_ids, std::vector<dReal>& friction_coeffs);
    void GetGIWC(std::vector<std::string>& manip_ids, std::vector<dReal>& friction_coeffs, std::vector<dReal>& ikparams);

    /// compute a support polygon based on what links are touching the ground
    void GetSupportPolygon(std::vector<string>& supportlinks, std::vector<dReal>& polyx, std::vector<dReal>& polyy, Vector polyscale = Vector(1.0,1.0,1.0), Vector polytrans = Vector(1.0,1.0,1.0));

    /// compute the distance from a line segment to a point
    void GetDistanceFromLineSegment(dReal cx, dReal cy, dReal ax, dReal ay,
					  dReal bx, dReal by, dReal& distanceSegment,
					  dReal& xout, dReal& yout);
    /// get the camera transform
    bool GetCamView(ostream& sout, istream& sinput);

    /// set the camera transform
    bool SetCamView(ostream& sout, istream& sinput);

    /// worker function that launches the planner (used if calling cbirrt in non-blocking mode)
    void PlannerWorker(PlannerBasePtr _pTCplanner, TrajectoryBasePtr ptraj, string filename);

    /// writes out a trajectory generated by the planner
    void WriteTraj(TrajectoryBasePtr ptraj, string filename);

    /// returns the current state of the planner
    int GetPlannerState(ostream& sout, istream& sinput);

    /// stops the planner if it is running
    int StopPlanner(ostream& sout, istream& sinput);

    /// clears objects drawn by cbirrt planner and problem
    bool ClearDrawn(ostream& sout, istream& sinput);

    RobotBasePtr robot;
    string _strRRTPlannerName;

    string _strRobotName; ///< name of the active robot

    PlannerBasePtr _pPlanner;
    bool _reusePlanner;
    IkSolverBasePtr _pIkSolver;

    boost::shared_ptr<boost::thread> _plannerThread;
    enum PlannerState _plannerState;
    vector<KinBody::JointPtr> _limadj_joints;
    vector<vector<dReal> > _limadj_lowers;
    vector<vector<dReal> > _limadj_uppers;

    std::vector<string> prev_support_manips;
    std::vector<dReal> prev_giwc;

    std::map<string,NEWMAT::Matrix> _computed_contact_surface_cones;

};

#endif
