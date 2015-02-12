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
#ifndef SELECTABLE_ITEM_HPP
#define SELECTABLE_ITEM_HPP

#include <iostream>
#include <stdio.h>
#include <vector>

#include "selectable_item_table.hpp"

using namespace std;

class SelectableItemTable;

class SelectableItem
{
public:
    SelectableItem(string name, string id);
    ~SelectableItem();

    void set_id( const std::string id ){ id_ = id; } ;

    void set_selected(bool selected);
    void set_reference(SelectableItemTable* reference);

    bool            get_selected();
    vector<string>  get_types();

    void add_type(string type);

    SelectableItemTable* get_reference();
    string               get_name();
    string               get_id();

private:
    string                  name_;
    vector<string>          types_;
    string                  id_;
    bool                    selected_;
    SelectableItemTable*    reference_;
};

#endif // SELECTABLE_ITEM_HPP

