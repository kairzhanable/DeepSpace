using System.Collections.Generic;
using UnityEngine;

namespace DeepSpace
{
    public enum ModuleType 
    {
        EXTERNAL,
        INTERNAL,
        ENGINE
    }

    public class SystemElements<T>
    {
        private List<T> elements {get; set;}
        public SystemElements()
        {
            this.elements = new List<T>();
        }

        public bool Add(Module module)
        {
            if(module is T)
            {
                this.elements.Add((T)(object)module);
                return true;
            }
            return false;
        }

        public bool Remove(Module module)
        {
            if(module is T)
            {
                this.elements.Remove((T)(object)module);
                return true;
            }
            return false;
        }

        public int Length{
            get{ return elements.Count; }
        }

        public T get(int idx){
            return elements[idx];
        }
    }

    public static class Extension
    {
        public static List<T> GetComponentsInChildrenRecursively<T>(this Transform _transform, List<T> _componentList)
        {
            foreach (Transform t in _transform)
            {
                T[] components = t.GetComponents<T>();
                foreach (T component in components)
                {
                    if (component != null)
                    {
                        _componentList.Add(component);
                    }
                }
                GetComponentsInChildrenRecursively<T>(t, _componentList);
            }
            return _componentList;
        }
    }
}