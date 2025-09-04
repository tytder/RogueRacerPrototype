using UnityEngine;
using UnityUtils;

public class SceneButtonsManager : MonoBehaviour
{
    public bool IsGameWonScreen = false;
    public void OpenScene(string sceneName)
    {
        if (IsGameWonScreen)
        {
            GameSingleton.Instance.RacesCompleted = 0;
            GameSingleton.Instance.PlayerHandlingMultiplier = .9f;
            GameSingleton.Instance.PlayerAccelMultiplier = .9f;
            GameSingleton.Instance.PlayerSpeedMultiplier = .8f;
        }
        GameSingleton.Instance.OpenScene(sceneName);
    }
    
    public void QuitGame()
    {
        GameSingleton.Instance.QuitGame();
    }
}
