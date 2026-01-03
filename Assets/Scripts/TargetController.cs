using UnityEngine;
using UnityEngine.InputSystem; // For InputAction

public class TargetController : MonoBehaviour
{
    public float moveSpeed = 5f;

    [Header("Input System")]
    public InputAction moveAction;  // assign in Inspector

    private void OnEnable()
    {
        moveAction.Enable();
    }

    private void OnDisable()
    {
        moveAction.Disable();
    }

    void Update()
    {
        Vector2 input = moveAction.ReadValue<Vector2>();
        Vector3 move = new Vector3(input.x, input.y, 0f);
        transform.localPosition += move * moveSpeed * Time.deltaTime;
    }
}
