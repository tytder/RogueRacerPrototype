using System;
using System.Collections;
using System.Linq;
using TMPro;
using UnityEngine;
using UnityUtils;

public class RaceManager : Singleton<RaceManager>
{
    public int TotalAmountOfLaps = 3;
    [SerializeField] private TMP_Text RaceDoneScreen;
    [SerializeField] private TMP_Text _raceCountdown;
    [SerializeField] private int _raceCountdownTime = 5;
    [SerializeField] private string _rewardScreen;
    public bool IsRaceActive = false;

    private float _timer;
    
    private void Start()
    {
        _raceCountdown.transform.parent.gameObject.SetActive(true);
        _timer = _raceCountdownTime - .000001f;
    }

    private void Update()
    {
        if (!IsRaceActive)
        {
            _timer -= Time.deltaTime;
            if (_timer > 0f)
            {
                _raceCountdown.text = "" + ((int)_timer + 1);
            }
            if (_timer <= 0f && _raceCountdown.transform.parent.gameObject.activeInHierarchy)
            {
                IsRaceActive = true;
                _raceCountdown.transform.parent.gameObject.SetActive(false);
                return;
            }
        }
    }

    public IEnumerator ShowRaceCompletedMenu()
    {
        IsRaceActive = false;
        yield return new WaitForSeconds(1f);
        RaceDoneScreen.transform.parent.gameObject.SetActive(true);
        var cars = FindObjectsByType<CarController>(FindObjectsSortMode.None);
        var player = cars.FirstOrDefault(car => car.PlayerInput);
        if (player)
            RaceDoneScreen.text = $"Finished at:\n {player.RacePosition} / {cars.Length}";
    }
    
    public void FinishRace()
    {
        if (!GameSingleton.Instance.IncreaseCompletedRaces())
        {
            GameSingleton.Instance.OpenScene(_rewardScreen);
        }
    }

    public void RaceCompleted()
    {
        StartCoroutine(nameof(ShowRaceCompletedMenu));
    }
}