public interface ICarInput
{
    public float Move { get; }
    public float Steer { get; }

    public bool SetCurrentWaypoint(Waypoint currentWaypoint);
}