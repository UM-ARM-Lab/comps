#ifndef  CONTACTREGION_H
#define  CONTACTREGION_H


class ContactRegion
{
	public:
		ContactRegion(Vector p, Vector n, float r);
		DistToContactRegion(Transform contact_transform);
		TranslationDistToContactRegion(Transform contact_transform);
		OrientationDistToContactRegion(string contact_manip, Transform contact_transform);
		Vector position;
		Vector normal;
		float radius;
	private:

};

ContactRegion::TranslationDistToContactRegion(Transform contact_transform)
{
	float center_translation = contact_transform.trans - position;

	float z_dist = fabs(center_translation.dot(normal));

	float xy_dist = max(sqrt(pow(center_translation.lengthsqr3(),2) - pow(z_dist,2)) - radius, 0.0);

	float translation_dist = sqrt(pow(xy_dist,2) + pow(z_dist,2));

	return translation_dist;
}

ContactRegion::OrientationDistToContactRegion(string contact_manip, Transform contact_transform)
{
	contact_transform_matrix = RaveTransformMatrix(contact_transform);
	Vector contact_normal;

	if(strcmp(contact_manip,'l_arm') == 0 || strcmp(contact_manip,'r_arm') == 0)
	{
		contact_normal.x = contact_transform_matrix.m[0]
		contact_normal.y = contact_transform_matrix.m[4]
		contact_normal.z = contact_transform_matrix.m[8]
	}
	else if(strcmp(contact_manip,'l_leg') == 0 || strcmp(contact_manip,'r_leg') == 0)
	{
		contact_normal.x = -contact_transform_matrix.m[2]
		contact_normal.y = -contact_transform_matrix.m[6]
		contact_normal.z = -contact_transform_matrix.m[10]
	}

	float orientation_dist = 1 - fabs(contact_normal.dot(normal));

	return orientation_dist;
}

ContactRegion::DistToContactRegion(Transform contact_transform)
{
	translation_dist = TranslationDistToContactRegion(contact_transform);
	orientation_dist = OrientationDistToContactRegion(contact_transform);

	float k = 0.5;

	return (translation_dist + k * orientation_dist);
}

ContactRegion::ContactRegion(Vector p, Vector n, float r)
{
	position = p;
	normal = n;
	radius = r;
}

#endif