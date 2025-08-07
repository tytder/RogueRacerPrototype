using System;
using UnityEngine;
using UnityEngine.InputSystem;

[CreateAssetMenu(fileName = "InputReader", menuName = "Scriptable Objects/InputReader")]
public class InputReader : ScriptableObject, PlayerInputActions.IPlayerActions
{
    public float Move => playerInputActions.Player.Move.ReadValue<float>();
    public float Steer => playerInputActions.Player.Steer.ReadValue<float>();
    public bool IsBreaking => Move < 0f;
    
    PlayerInputActions playerInputActions;

    void OnEnable()
    {
        if (playerInputActions == null)
        {
            playerInputActions = new PlayerInputActions();
            playerInputActions.Player.SetCallbacks(this);
        }
        playerInputActions.Enable();
    }

    public void Enable()
    {
        playerInputActions.Enable();
    }

    private void OnDisable()
    {
        playerInputActions.Disable();
    }

    public void OnSteer(InputAction.CallbackContext context)
    {
    }

    public void OnMove(InputAction.CallbackContext context)
    { 
    }

    public void OnLookSideToSide(InputAction.CallbackContext context)
    {
        // noop
    }

    public void OnLookBehind(InputAction.CallbackContext context)
    {
        // noop
    }
}
