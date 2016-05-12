#include "Facet_Group.h"
#include "../ui/color_table.h"
Facet_Group::Facet_Group(void)
{
	facet_group_name_ = "";
	color_ = GLOBAL::random_color(1);
}

Facet_Group::~Facet_Group(void)
{
}
