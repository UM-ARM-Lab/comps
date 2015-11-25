#ifndef  BALANCE_H
#define  BALANCE_H

#include <cdd/setoper.h>
#include <cdd/cdd.h>

enum BalanceMode {
    BALANCE_NONE,
    BALANCE_SUPPORT_POLYGON,
    BALANCE_GIWC
};

class Balance
{
	public:
		Balance(RobotBasePtr r, std::vector<string> s_links, Vector p_scale, Vector p_trans);
		Balance(RobotBasePtr r, Vector g, std::vector<string> s_manips, std::vector<dReal> s_mus);
		~Balance(){};
		void RefreshBalanceParameters(std::vector<dReal> q_new); // call it after the robot is in a new configuration, and before calling CheckSupport. Takes new config.
		bool CheckSupport(Vector center);
    	
	private:
		BalanceMode balance_mode;
		RobotBasePtr robot;

		//support polygon
		std::vector<string> supportlinks;
		Vector polyscale;
		Vector polytrans;
		std::vector<dReal> supportpolyx;
    	std::vector<dReal> supportpolyy;

    	//GIWC
		Vector gravity;
		std::vector<string> support_manips;
		std::vector<dReal> support_mus;
		NEWMAT::Matrix giwc;

		void InitializeBalanceParameters();

		void GetSupportPolygon();
		
		void GetGIWC();
		
		NEWMAT::ReturnMatrix GetGIWCSpanForm();	
		NEWMAT::ReturnMatrix GetSurfaceCone(string& manipname, dReal mu);
		void GetSupportPointsForLink(RobotBase::LinkPtr p_link, OpenRAVE::Vector tool_dir, Transform result_tf, std::vector<Vector>& contacts);
    	std::vector<Vector> GetSupportPoints(RobotBase::ManipulatorPtr p_manip);
    	void GetFrictionCone(OpenRAVE::Vector &center, OpenRAVE::Vector &direction, dReal mu, NEWMAT::Matrix *mat, int offset_r, int offset_c, Transform temp_tf);
    	void GetASurf(RobotBase::ManipulatorPtr p_manip, Transform cone_tf, NEWMAT::Matrix *mat, int offset_r);
    	void GetAStance(Transform cone_tf, NEWMAT::Matrix* mat, int offset_r);


};

#endif
