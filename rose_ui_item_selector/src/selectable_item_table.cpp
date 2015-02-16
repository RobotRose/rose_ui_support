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

#include "rose_ui_item_selector/selectable_item_table.hpp"

SelectableItemTable::SelectableItemTable()
    : selection_mode_ ( eSingleSelection)
    , last_selection_ ( 0 )
{
}

SelectableItemTable::~SelectableItemTable()
{

}

void SelectableItemTable::addItem(SelectableItem* item)
{
    itemlist_.push_back(item);
}

void SelectableItemTable::set_reference(SelectableItem* item)
{
    reference_ = item;
}

vector<SelectableItem*> SelectableItemTable::get_itemlist()
{
    return itemlist_;
}

vector<string> SelectableItemTable::toString()
{
    vector<string> result;
    for ( vector<SelectableItem*>::iterator it = itemlist_.begin() ; it != itemlist_.end() ; it++ )
        result.push_back((*it)->get_name());

    return result; 
}

void SelectableItemTable::set_selection_mode( eSelectionMode selection_mode )
{
    selection_mode_ = selection_mode;
}

void SelectableItemTable::setNoSelection()
{
    for ( int i = 0 ; i < itemlist_.size(); i++)
        itemlist_.at(i)->set_selected( false );
}

void SelectableItemTable::itemSelected(int row_nr)
{
    switch ( selection_mode_ )
    {
        case ( eNoSelection ):
            setNoSelection();
        break;

        case( eSingleSelection ):
            setNoSelection();
            itemlist_.at(row_nr)->set_selected( true );
        break;

        case ( eMultiSelection ):
            itemlist_.at(row_nr)->set_selected( !itemlist_.at(row_nr)->get_selected() );
        break;

        default:
        break;
    }

    last_selection_ = row_nr;
}

eSelectionMode SelectableItemTable::get_selection_mode()
{
    return selection_mode_;
}

int SelectableItemTable::get_last_selection()
{
    return last_selection_;
}

SelectableItem* SelectableItemTable::getSelectableItemById( string id )
{
    for ( vector<SelectableItem*>::iterator it = itemlist_.begin() ; it != itemlist_.end() ; it++ )
        if ( (*it)->get_id() == id )
            return *it;
}

SelectableItem* SelectableItemTable::getSelectableItem( int i )
{
    if ( i < 0 || i >= itemlist_.size() ) 
        return NULL;

    return itemlist_[i];
}

void SelectableItemTable::removeItem( int i )
{
    if ( i < 0 || i >= itemlist_.size() ) 
        return;

    SelectableItem* item = getSelectableItem(i);

    // remove from list and remove the pointer
    itemlist_.erase(itemlist_.begin()+i);
    delete item;

    return;
}

vector<int> SelectableItemTable::getSelectedItemPositions()
{
    vector<int> result;

    for ( int i = 0 ; i < itemlist_.size() ; i++ )
        if ( itemlist_.at(i)->get_selected() )
            result.push_back(i);

    return result;
}

vector<string> SelectableItemTable::getSelectedItemIds()
{
    vector<string> result;

    for ( auto item = itemlist_.begin() ; item != itemlist_.end() ; item++ )
        if ( (*item)->get_selected() )
            result.push_back((*item)->get_id());

    return result;
}


void SelectableItemTable::sortItems()
{
    std::sort(itemlist_.begin(), itemlist_.end(), &SelectableItemTable::compare);
}

bool SelectableItemTable::compare(SelectableItem* lhs, SelectableItem* rhs) 
{ 
    return lhs->get_name() < rhs->get_name(); 
};