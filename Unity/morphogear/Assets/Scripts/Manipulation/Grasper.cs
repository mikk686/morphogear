using UnityEngine;
using static UnityEngine.GraphicsBuffer;

public class Grasper : MonoBehaviour
{
    private Collider objectInsideCollider = null;

    public string dedicatedTag = "Grabbable";

    public void GrabObject(bool grasp)
    {
        if (grasp)
        {
            if ((objectInsideCollider != null) && (objectInsideCollider.CompareTag(dedicatedTag)))
            {
                objectInsideCollider.transform.position = transform.position;
                objectInsideCollider.transform.SetParent(this.transform);
            }
        }
        else
        {
            if ((objectInsideCollider != null)&&(objectInsideCollider.CompareTag(dedicatedTag)))
            {
                objectInsideCollider.transform.position = transform.position;
                objectInsideCollider.transform.SetParent(null);
            }
        }

    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag(dedicatedTag))
            objectInsideCollider = other; // Remember the object that entered
    }

    void OnTriggerExit(Collider other)
    {
        if (other == objectInsideCollider)
        {
            objectInsideCollider = null; // Forget the object when it leaves
        }
    }
}
