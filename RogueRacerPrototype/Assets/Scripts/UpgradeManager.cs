
using UnityEngine;
using UnityUtils;

public class UpgradeManager : Singleton<UpgradeManager>
{
    public void UpgradeSpeed()
    {
        GameSingleton.Instance.PlayerSpeedMultiplier *= 1.2f;
        GameSingleton.Instance.OpenScene(GameSingleton.Instance.RaceSelectScreen);
    }

    public void UpgradeAccel()
    {
        GameSingleton.Instance.PlayerAccelMultiplier *= 1.2f;
        GameSingleton.Instance.OpenScene(GameSingleton.Instance.RaceSelectScreen);
    }

    public void UpgradeHandling()
    {
        GameSingleton.Instance.PlayerHandlingMultiplier *= 1.2f;
        GameSingleton.Instance.OpenScene(GameSingleton.Instance.RaceSelectScreen);
    }
}
