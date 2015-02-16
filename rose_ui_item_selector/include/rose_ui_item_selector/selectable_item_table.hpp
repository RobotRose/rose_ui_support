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

#ifndef SELECTABLE_ITEM_TABLE_HPP
#define SELECTABLE_ITEM_TABLE_HPP

#include <ros/ros.h>
#include <stdio.h>
#include <vector>

#include "selectable_item.hpp"
#include "rose_ui_item_selector/selection_mode.hpp"

class SelectableItem;

class SelectableItemTable
{
public:

    SelectableItemTable();
    ~SelectableItemTable();

    void addItem( SelectableItem* item );
    
    template<class T>
    void addItems( std::vector< T > items ) 
    { 
        for ( auto item = items.begin() ; item != items.end() ; item++ ) 
        {
            SelectableItem* si = new SelectableItem((*item).get_name(), (*item).get_id());
            addItem(si); 
        }
    };
    
    void sortItems();
    static bool compare(SelectableItem* lhs, SelectableItem* rhs);
   
    void set_reference ( SelectableItem* item );
    void set_selection_mode ( eSelectionMode selection_mode );
    void itemSelected( int row_nr );
    void setNoSelection();

    std::vector<SelectableItem*>  get_itemlist();
    eSelectionMode      get_selection_mode();
    int                 get_last_selection();

    std::vector<std::string>      toString();
    SelectableItem*     getSelectableItem( int i );
    SelectableItem*     getSelectableItemById( std::string id );
    std::vector<int>         getSelectedItemPositions();
    std::vector<std::string>      getSelectedItemIds();
    void                removeItem( int i );

private:
    std::vector<SelectableItem*>  itemlist_;
    SelectableItem*          reference_;
    eSelectionMode      selection_mode_;
    int                 last_selection_;
};

#endif // SELECTABLE_ITEM_TABLE_HPP
