/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2014/01/14
*         - File created.
*
* Description:
*    description
* 
***********************************************************************************/

#include "selectable_item.hpp"

SelectableItem::SelectableItem( string name, string id )
    : id_ ( id )
    , name_ ( name )
    , selected_ ( false )
    , reference_ ( NULL )
{
}

SelectableItem::~SelectableItem()
{

}

void SelectableItem::set_selected(bool selected)
{
    selected_ = selected;
}

bool SelectableItem::get_selected()
{
    return selected_;
}

vector<string> SelectableItem::get_types()
{
    return types_;
}

void SelectableItem::set_reference(SelectableItemTable* reference)
{
    reference_ = reference;
}

void SelectableItem::add_type(string type)
{
    types_.push_back(type);
}

SelectableItemTable* SelectableItem::get_reference()
{
    return reference_;
}

string SelectableItem::get_name()
{
    return name_;
}

string SelectableItem::get_id()
{
    return id_;
}
