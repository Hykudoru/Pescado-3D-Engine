#pragma once
#include <Graphics.h>
class Collider;
class PhysicsObject;

template <typename T>
class TreeNode : public CubeMesh
{
public:
    Vec3 min_w;
    Vec3 max_w;
    int level = 0;
    static int maxDepth;
    static int count;
    int maxCapacity = 8;
    TreeNode<T>* root = nullptr;
    TreeNode<T>* subroot = nullptr;
    TreeNode<T>* parent = nullptr;
    List<T*> contained = List<T*>();
    List<TreeNode<T>*>* children = nullptr;

    TreeNode(TreeNode<T>* parent = nullptr)
    {
        count++;
        if (!parent)
        {
            root = this;
            localScale = Vec3(10000, 10000, 10000);
        }
        else
        {
            this->parent = parent;
            root = parent->root;
            level = parent->level + 1;
            localScale = (parent->localScale * .5);
            if (level == 1) {
                subroot = this;
            }
            if (level > 1) {
                SetColor(parent->color);
            }
        }

        SetVisibility(false);
    }

    virtual ~TreeNode<T>()
    {
        if (children)
        {
            for (size_t i = 0; i < children->size(); i++)
            {
                delete (*children)[i];
            }

            delete children;
        }
    }

    void Subdivide()
    {
        if (!children && level < maxDepth)
        {
            children = new List<TreeNode<T>*>();

            for (int width = -1; width <= 1; width += 2)
            {
                for (int height = -1; height <= 1; height += 2)
                {
                    for (int depth = -1; depth <= 1; depth += 2)
                    {
                        auto newChild = new TreeNode<T>(this);
                        newChild->localPosition = Position() + Direction::right * width * .5 * newChild->localScale.x + Direction::up * height * .5 * newChild->localScale.y + Direction::forward * depth * .5 * newChild->localScale.z;
                        newChild->bounds->CreateBounds(newChild);
                        newChild->min_w = newChild->TRS() * newChild->bounds->min;
                        newChild->max_w = newChild->TRS() * newChild->bounds->max;

                        children->emplace_back(newChild);
                    }
                }
            }
        }
    }

    void ForEachSubNode(const std::function<void(TreeNode<T>*)>& action)
    {
        if (children)
        {
            for (size_t i = 0; i < children->size(); i++)
            {
                action((*children)[i]);
                (*children)[i]->ForEachSubNode(action);
            }
        }
    }

    bool OverlappingPoint(Vec3& point)
    {
        if (point.x >= min_w.x && point.x <= max_w.x
            && point.y >= min_w.y && point.y <= max_w.y
            && point.z >= min_w.z && point.z <= max_w.z)
        {
            return true;
        }

        return false;
    }
    
    bool Overlapping(T* obj)
    {
        List<Vec3>* verts;
        auto func = [&]()
            {
                for (size_t i = 0; i < 8; i++)
                {
                    if (!this->OverlappingPoint((*verts)[i]))
                    {
                        return false;
                    }
                }
                return true;
            };

        //if atleast containing position, check if completely overlapping bounds.
        Vec3 pos = obj->Position();
        if (this->OverlappingPoint(pos))
        {
            Mesh* mesh = dynamic_cast<Mesh*>(obj);
            if (mesh)
            {
                verts = mesh->bounds->WorldVertices();
                return func();
            }

            PhysicsObject* physObj = dynamic_cast<PhysicsObject*>(obj);
            if (physObj)
            {
                verts = physObj->mesh->bounds->WorldVertices();
                return func();
            }

            Collider* collider = dynamic_cast<Collider*>(obj);
            if (collider)
            {
                verts = collider->object->mesh->bounds->WorldVertices();
                return func();
            }
        }

        return false;
    }

    TreeNode<T>* Query(Vec3& pos, List<T*>& containerFilling, const std::function<bool(T*)>& condition = NULL)
    {
        if (this->OverlappingPoint(pos))
        {
            //cout << " >> level: " << this->level;
            this->Extract(containerFilling, condition);

            if (!this->children)
            {
                return this;
            }
            else
            {
                for (size_t i = 0; i < this->children->size(); i++)
                {
                    auto node = (*children)[i];
                    node = node->Query(pos, containerFilling, condition);
                    if (node != nullptr) {
                        return node;
                    }
                }
            }
        }

        return nullptr;
    }

    TreeNode<T>* Insert(T* obj)
    {
        // Prevent box from being inserted into itself.
        if (obj == (T*)this)
        {
            return nullptr;
        }

        if (Overlapping(obj))
        {
            if (contained.size() < maxCapacity)
            {
                contained.emplace_back(obj);
                return this;
            }
            else
            {
                if (!children && level < maxDepth)
                {
                    Subdivide();
                }

                if (children)
                {
                    TreeNode<T>* node = nullptr;
                    for (size_t i = 0; i < children->size(); i++)
                    {
                        node = (*children)[i];
                        node = node->Insert(obj);
                        if (node) {
                            return node;
                        }
                    }
                    if (!node)
                    {
                        // Insert into parent node since no children could fully contain it.
                        contained.emplace_back(obj);
                        return this;
                    }
                }
            }
        }

        return nullptr;
    }

    void Extract(List<T*>& containerFilling, const std::function<bool(T*)>& condition = NULL)
    {
        if (condition)
        {
            for (size_t i = 0; i < this->contained.size(); i++)
            {
                auto obj = this->contained[i];
                if (condition(obj)) {
                    containerFilling.emplace_back(obj);
                }
            }
        }
        else
        {
            for (size_t i = 0; i < this->contained.size(); i++)
            {
                containerFilling.emplace_back(this->contained[i]);
            }
        }
    }

    void ExtractAll(List<T*>& containerFilling)
    {
        this->Extract(containerFilling);
        
        if (this->children)
        {
            for (size_t i = 0; i < this->children->size(); i++)
            {
                (*this->children)[i]->Extract(containerFilling);
            }
        }
    }

    void Draw()
    {
        if (!Graphics::debugTree)
        {
            return;
        }

        if (this->level > 0)
        {
            this->SetVisibility(true);
        }
        else {
            this->SetVisibility(false);
        }

        this->bounds->color = color;

        //Point::AddWorldPoint(Point(this->Position(), this->color, 15));
        //Point::AddWorldPoint(Point(min_w, Color::blue, 10));
        //Point::AddWorldPoint(Point(max_w, Color::blue, 10));
        //Line::AddWorldLine(Line(min_w, max_w, Color::pink, 5));

        for (size_t i = 0; i < this->contained.size(); i++)
        {
            auto obj = this->contained.at(i);
            Point::AddWorldPoint(Point(obj->Position(), this->color, 8));
        }

        if (this->children)
        {
            for (size_t i = 0; i < this->children->size(); i++)
            {
                auto child = (*this->children)[i];
                child->Draw();
            }
        }
    }
};

template <typename T>
int TreeNode<T>::maxDepth = 12;
template <typename T>
int TreeNode<T>::count = 0;

template <typename T>
class OctTree : public TreeNode<T>
{
private:
    static OctTree<T>* tree;
public:
    OctTree() : TreeNode<T>()
    {
        this->Subdivide();

        // Color zones
        (*this->children)[0]->SetColor(Color::red);
        (*this->children)[1]->SetColor(Color::orange);
        (*this->children)[2]->SetColor(Color::yellow);
        (*this->children)[3]->SetColor(Color::green);
        (*this->children)[4]->SetColor(Color::blue);
        (*this->children)[5]->SetColor(Color::purple);
        (*this->children)[6]->SetColor(Color::pink);
        (*this->children)[7]->SetColor(Color::turquoise);

        // Insert world objects
        for (size_t i = 0; i < ManagedObjectPool<T>::objects.size(); i++)
        {
            T* objTesting = ManagedObjectPool<T>::objects[i];
            bool inserted = false;

            for (size_t ii = 0; ii < this->children->size(); ii++)
            {
                auto zone = (*this->children)[ii];
                auto node = zone->Insert(objTesting);
                if (node) {
                    inserted = true;
                    break;
                }
                /*// Double check if subroot is overlapping and insert here instead of main root 
                // since insertion may have failed due to size in some smaller branch node.
                else if (zone->Overlapping(objTesting))
                {
                    // Prevent box from being inserted into itself.
                    if (objTesting != (T*)zone)
                    {
                        inserted = true;
                        zone->contained.emplace_back(objTesting);
                        break;
                    }
                }*/
            }

            if (!inserted)
            {
                // Prevent box from being inserted into itself.
                if (objTesting != (T*)this)
                {
                    //objects too big or not encapsulated
                    this->contained.emplace_back(objTesting);
                }
            }
        }
    }

    static OctTree<T>* Tree()
    {
        if (!tree)
        {
            tree = new OctTree<T>();
        }

        return tree;
    }

    static void Update()
    {
        if (tree) {
            delete tree;
        }
        OctTree<T>::count = 0;
        tree = new OctTree<T>();

        tree->Draw();
    }

    static List<T*>* Search(Cube& volume, const std::function<void(T*)>& action = NULL)
    {
        static List<T*> list = List<T*>();
        list.clear();

        //Extract contents from root which contains any objects too big or not encapsulated
        Tree()->Extract(list);

        for (size_t i = 0; i < 8; i++)
        {
            Vec3 point = volume.vertices[i];
            //Query subnodes
            for (size_t ii = 0; ii < 8; ii++)
            {
                auto node = (*tree->children)[ii]->Query(point, list);
                if (node) {
                    //cout << endl;
                    break;
                }
            }
        }

        if (action)
        {
            for (size_t i = 0; i < list.size(); i++)
            {
                action(list[i]);
            }
        }

        return &list;
    }
    
    static List<T*>* Search(Vec3&& point, const std::function<void(T*)>& action = NULL)
    {
        static List<T*> list = List<T*>();
        list.clear();

        //Extract contents from root which contains any objects too big or not encapsulated
        Tree()->Extract(list);

        //Query subnodes
        for (size_t i = 0; i < 8; i++)
        {
            auto node = (*tree->children)[i]->Query(point, list);
            if (node) {
                //cout << endl;
                break;
            }
        }

        if (action)
        {
            for (size_t i = 0; i < list.size(); i++)
            {
                action(list[i]);
            }
        }

        return &list;
    }

    static List<T*> ExtractZone(int zoneID)
    {
        List<T*> list;

        if (zoneID > 7 || zoneID < 0)
        {
            return list;
        }

        auto zone = (*Tree()->children)[zoneID];
        zone->ExtractAll(list);

        return list;
    }
};
template <typename T>
OctTree<T>* OctTree<T>::tree = nullptr;
