using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.Serialization;
using UnityUtils;

public class GameSingleton : PersistentSingleton<GameSingleton>
{
    [FormerlySerializedAs("_amountOfRaces")] public int AmountOfRaces;
    [SerializeField] private string GameWonScene;
    public string RaceSelectScreen;
        
    [HideInInspector] public int RacesCompleted;
    [HideInInspector] public float PlayerHandlingMultiplier = .9f;
    [HideInInspector] public float PlayerAccelMultiplier = .9f;
    [HideInInspector] public float PlayerSpeedMultiplier = .8f;

    public void OpenScene(string sceneName)
    {
        SceneManager.LoadScene(sceneName);
    }

    public void QuitGame()
    {
        Application.Quit();
    }

    public bool IncreaseCompletedRaces()
    {
        RacesCompleted++;
        if (RacesCompleted == AmountOfRaces)
        {
            OpenScene(GameWonScene);
        }

        return RacesCompleted == AmountOfRaces;
    }
}
