using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RaceSelectMenu : MonoBehaviour
{
    public Button[] raceButtonLocks;
    [SerializeField] private string _raceName;
    private void OnEnable()
    {
        for (var index = 0; index < raceButtonLocks.Length; index++)
        {
            var button = raceButtonLocks[index];
            if (index > GameSingleton.Instance.RacesCompleted)
            {
                button.interactable = false;
            }
        }
    }

    public void OpenRace(int raceId)
    {
        if (raceId != GameSingleton.Instance.RacesCompleted) return;
        GameSingleton.Instance.OpenScene(_raceName);
    }
}
