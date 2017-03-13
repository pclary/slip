#pragma once

#include <vector>
#include <cstddef>


template <class T>
class Tree {
public:
    Tree<T>();
    Tree<T>(T data);
    ~Tree<T>();

    T data;
    Tree<T>* const parent;
    const size_t depth;

    std::vector<Tree<T>*> children;
    Tree<T>* add_child(T data);
    void remove_child(size_t i);
    void clear();

    size_t size();

private:
    Tree<T>(T data, Tree<T>* parent, size_t depth);

};


template <class T>
Tree<T>::Tree() :
    data(T()), parent(nullptr), depth(0)
{
}


template <class T>
Tree<T>::Tree(T data) :
    data(data), parent(nullptr), depth(0)
{
}


template <class T>
Tree<T>::~Tree()
{
    for (auto c : children)
        delete c;
}


template <class T>
Tree<T>::Tree(T data, Tree<T>* parent, size_t depth) :
    data(data), parent(parent), depth(depth)
{
}


template <class T>
Tree<T>* Tree<T>::add_child(T data)
{
    auto c = new Tree<T>(data, this, depth + 1);
    children.push_back(c);
    return c;
}


template <class T>
void Tree<T>::remove_child(size_t i)
{
    delete children[i];
    children.erase(children.begin() + i);
}


template <class T>
void Tree<T>::clear()
{
    children.clear();
}


template <class T>
size_t Tree<T>::size()
{
    size_t s = 1;
    for (auto& c : children)
        s += c->size();
    return s;
}
