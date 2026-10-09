using Priority_Queue;
using System.Collections;
using System.Collections.Immutable;
using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text.Json;
using System.Windows;
using System.Windows.Media;

namespace Jump_Bruteforcer
{
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct SearchNode
    {
        private const int XMask = 0x3ff;
        private const int FlagsShift = 10;

        private readonly double y;
        private readonly double vSpeed;
        private readonly int packedXAndFlags;
        private readonly int nodeIndex;

        public SearchNode(State state, int nodeIndex)
        {
            Debug.Assert((uint)state.X <= XMask);
            y = state.Y;
            vSpeed = state.VSpeed;
            packedXAndFlags = state.X | ((int)(byte)state.Flags << FlagsShift);
            this.nodeIndex = nodeIndex;
        }

        public int X => packedXAndFlags & XMask;
        public double Y => y;
        public int RoundedY => (int)Math.Round(y);
        public Bools Flags => (Bools)(byte)(packedXAndFlags >> FlagsShift);
        public int NodeIndex => nodeIndex;
        public State State => new() { X = X, Y = y, VSpeed = vSpeed, Flags = Flags };

        public bool IsGoal((int x, int y) goal) => Math.Abs(X - goal.x) <= 1 & RoundedY == goal.y;

        /// <summary>
        /// The heuristic contribution that goes into the priority. With a weight above one the
        /// search turns greedy: it finds a solution much faster at the cost of optimality, which
        /// is what the layered search uses to obtain an upper bound.
        /// </summary>
        internal static uint ScaledDistance(uint distance, int weight) =>
            distance == uint.MaxValue || weight == 1
                ? distance
                : (uint)Math.Min(uint.MaxValue - 1u, (ulong)distance * (ulong)weight);

        internal static uint DecodePathCost(ulong priority, uint distance, int weight) =>
            unchecked((uint)(priority >> 32) - ScaledDistance(distance, weight));
    }

    public class Search : INotifyPropertyChanged
    {
        private (int x, double y) start;
        private (int x, int y) goal;
        private string _strat = "";
        private bool disableCactus = false;
        private int nudgePenalty = 1;
        private CollisionMap _collisionMap = new(new Dictionary<(int, int), CollisionType>(), null);
        private PointCollection playerPath = new();
        private double startingVSpeed = 0;
        private String nodesVisited = "";
        private String timeTaken = "";
        private String macro = "";
        public PointCollection PlayerPath { get { return playerPath; } set { playerPath = value; OnPropertyChanged(); } }
        public int StartX { get { return start.x; } set { start.x = value; OnPropertyChanged(); } }
        public double StartY { get { return start.y; } set { start.y = value; OnPropertyChanged(); } }
        public int GoalX { get { return goal.x; } set { goal.x = Math.Clamp(value, 0, Map.WIDTH - 1); OnPropertyChanged(); } }
        public int GoalY { get { return goal.y; } set { goal.y = Math.Clamp(value, 0, Map.HEIGHT - 1); OnPropertyChanged(); } }
        public string Strat { get { return _strat; } set { _strat = value; OnPropertyChanged(); } }
        public String NodesVisited { get { return nodesVisited; } set { nodesVisited = value; OnPropertyChanged(); } }
        public CollisionMap CollisionMap { get { return _collisionMap; } set { _collisionMap = value; } }
        public double StartingVSpeed { get { return startingVSpeed; } set { startingVSpeed = value; OnPropertyChanged(); } }
        /// <summary>Weight applied to the heuristic. 1 keeps the search optimal, higher values make it greedy.</summary>
        public int HeuristicWeight { get; set; } = 1;
        /// <summary>Run the layered grouped BFS instead of the node based A*.</summary>
        /// <summary>Collect the per pixel state counts used by the heat map (costs one random write per state).</summary>
        public static bool CollectHeatMap = true;
        public bool UseLayeredBfs { get; set; } = false;
        /// <summary>Use the admissible lower bound instead of the historical flood fill heuristic.</summary>
        public bool UseAdmissibleHeuristic { get; set; } = false;
        /// <summary>Add the vertical pattern database to the heuristic.</summary>
        public bool UseVerticalBound { get; set; } = false;
        /// <summary>Let the layered search take its frame bound from a node based A* run first.</summary>
        public bool LayeredUsesAStarBound { get; set; } = true;
        /// <summary>Starting beam width of the bound pass of the layered search.</summary>
        public int BeamWidth { get; set; } = 512;
        /// <summary>Simulations performed by the last layered search.</summary>
        public long LayeredSimulations { get; private set; }
        /// <summary>Upper bound found by the beam pass, or uint.MaxValue.</summary>
        public uint LayeredUpperBound { get; private set; } = uint.MaxValue;
        public bool LayeredBeamSucceeded { get; private set; }
        /// <summary>Wall clock time of the last layered search.</summary>
        public TimeSpan LayeredElapsed { get; private set; }
        /// <summary>
        /// When set, a jump key release is only simulated while the key is still held, so the search
        /// can no longer use the cactus technique (releasing one press of the jump key more than once).
        /// </summary>
        public bool DisableCactus { get { return disableCactus; } set { disableCactus = value; OnPropertyChanged(); } }
        /// <summary>
        /// Enables the A/D nudge (d-trick): while the kid stands on a block or platform, pressing
        /// A or D shifts the horizontal speed by one pixel for that frame. Only some engines have
        /// this mechanic, so it is off by default.
        /// </summary>
        public bool AllowNudge { get { return Player.AllowNudge; } set { Player.AllowNudge = value; OnPropertyChanged(); } }
        /// <summary>
        /// Extra cost of a frame that uses the A/D nudge. 0 is the fastest sequence, 1 makes an
        /// A/D frame pay for itself, 2 keeps A/D for the screens that cannot be solved without it.
        /// A nudge frame walks 4px instead of 3, so without a penalty the search spends one every
        /// frame, which no player can press. Has no effect while AllowNudge is off.
        /// </summary>
        public int NudgePenalty { get { return nudgePenalty; } set { nudgePenalty = Math.Max(0, value); OnPropertyChanged(); } }
        public String TimeTaken { get { return timeTaken; } set { timeTaken = value; OnPropertyChanged(); } }
        public String Macro { get { return macro; } set { macro = value; } }
        // Numeric timings exclude map loading and result rendering/export.
        public TimeSpan FloodFillElapsed { get; private set; }
        public TimeSpan SearchElapsed { get; private set; }
        public int VisitedPlaneCount { get; private set; }
        public long VisitedBitmapBytes { get; private set; }
        public int VisitedOverflowCount { get; private set; }
        public int PathLinkCount { get; private set; }
        public long PathLinkPackedBytes { get; private set; }
        public int PathLinkWideCount { get; private set; }
        public long PathLinkWideBytes { get; private set; }
        public long PathLinkTotalBytes { get; private set; }
        public event PropertyChangedEventHandler? PropertyChanged;


        private void OnPropertyChanged([System.Runtime.CompilerServices.CallerMemberName] string propertyName = "")
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        public Search((int, double) start, (int, int) goal, CollisionMap collision)
        {
            this.start = start;
            this.goal = goal;
            CollisionMap = collision;
        }


        //inadmissable heuristic because of y position rounding
        public uint Distance(PlayerNode n)
        {
            uint packed = pixelInfo[PixelIndex(n.State.X, (int)Math.Round(n.State.Y))];
            uint distance = packed & HeuristicMask;
            return distance == HeuristicUnreachable ? uint.MaxValue : distance;
        }

        private AdmissibleDistance.Table? admissibleTable;
        /// <summary>The inputs of the last solution the node search found, kept so the layered
        /// search can compare its own solution against it.</summary>
        private List<Input> lastInputs = new();

        /// <summary>How many runs the notation of this path is written in.</summary>
        internal static int InputChanges(List<Input> inputs)
        {
            int runs = 0;
            for (int i = 0; i < inputs.Count; i++)
            {
                if (i == 0 || inputs[i] != inputs[i - 1]) runs++;
            }
            return runs;
        }

        public readonly uint[,] GoalDistance = new uint[Map.WIDTH, Map.HEIGHT];
        /// <summary>
        /// Heuristic value in the low 16 bits and the explored state counter in the high 16, packed
        /// so that the hot loop touches one cache line per discovered state instead of two.
        /// </summary>
        private readonly uint[] pixelInfo = new uint[Map.WIDTH * Map.HEIGHT];

        private const uint HeuristicMask = 0xFFFF;
        private const uint CounterStep = 1u << 16;
        private const uint HeuristicUnreachable = 0xFFFF;

        public void FloodFill()
        {
            HashSet<(int, int)> CurrentGoalPixels;
            if (goal.x + 1 == Map.WIDTH)
            {
                CurrentGoalPixels = new() { { goal }, { (goal.x - 2, goal.y) }, { (goal.x - 1, goal.y) } };
            }
            else if (goal.x - 1 < 0)
            {
                CurrentGoalPixels = new() { { goal }, { (goal.x + 2, goal.y) }, { (goal.x + 1, goal.y) } };
            }
            else
            {
                CurrentGoalPixels = new() { { goal }, { (goal.x + 1, goal.y) }, { (goal.x - 1, goal.y) } };
            }

            for (int X = 0; X < Map.WIDTH; X++)
            {
                for (int Y = 0; Y < Map.HEIGHT; Y++)
                {
                    GoalDistance[X, Y] = uint.MaxValue;
                    pixelInfo[PixelIndex(X, Y)] = HeuristicUnreachable;
                }
            }

            HashSet<(int X, int Y)> NewPositions = new(), Temp;

            foreach ((int X, int Y) GoalPos in CollisionMap.goalPixels.Union(CurrentGoalPixels))
            {
                GoalDistance[GoalPos.X, GoalPos.Y] = 0;
                pixelInfo[PixelIndex(GoalPos.X, GoalPos.Y)] = 0;
                NewPositions.Add(GoalPos);
            }

            int MaxHSpeed = PhysicsParams.WALKING_SPEED, MaxVSpeedDown = (int)Math.Ceiling(PhysicsParams.MAX_VSPEED + PhysicsParams.GRAVITY), MaxVSpeedUp = (int)Math.Abs(Math.Ceiling(PhysicsParams.SJUMP_VSPEED + PhysicsParams.GRAVITY));
            uint Distance = 1;

            while (NewPositions.Count > 0)
            {
                Temp = new HashSet<(int X, int Y)>(NewPositions);
                NewPositions.Clear();

                // floodfill
                foreach ((int X, int Y) Pos in Temp)
                {
                    int MinX = Math.Max(Pos.X - MaxHSpeed, 0),
                        MaxX = Math.Min(Pos.X + MaxHSpeed, Map.WIDTH - 1),
                        MinY = Math.Max(Pos.Y - MaxVSpeedDown, 0),
                        MaxY = Math.Min(Pos.Y + MaxVSpeedUp, Map.HEIGHT - 1);

                    for (int X = MinX; X <= MaxX; X++)
                    {
                        for (int Y = MinY; Y <= MaxY; Y++)
                        {
                            if (GoalDistance[X, Y] == uint.MaxValue && !(CollisionMap.Collision[X, Y].HasFlag(CollisionType.Killer) || CollisionMap.Collision[X, Y].HasFlag(CollisionType.Solid)))
                            {
                                GoalDistance[X, Y] = Distance;
                                pixelInfo[PixelIndex(X, Y)] = Distance;
                                NewPositions.Add((X, Y));
                            }
                        }
                    }
                }

                Distance++;
            }
        }


        public SearchResult RunAStar() => UseLayeredBfs ? RunLayered() : RunNodeSearch();

        /// <summary>
        /// Layered grouped BFS. Optimal, and it shares one vertical simulation between every state
        /// that has the same exact vertical state, which is where most of the speedup comes from.
        /// </summary>
        public SearchResult RunLayered()
        {
            var startTime = Stopwatch.GetTimestamp();
            FloodFillElapsed = TimeSpan.Zero;
            SearchElapsed = TimeSpan.Zero;
            VisitedPlaneCount = 0;
            VisitedBitmapBytes = 0;
            VisitedOverflowCount = 0;
            PathLinkCount = 0;
            PathLinkPackedBytes = 0;
            PathLinkWideCount = 0;
            PathLinkWideBytes = 0;
            PathLinkTotalBytes = 0;
            LayeredUpperBound = uint.MaxValue;
            LayeredBeamSucceeded = false;
            LayeredSimulations = 0;

            PlayerNode root = new PlayerNode(start.x, start.y, startingVSpeed, RootFlags(DisableCactus));
            // The layered search uses its own admissible table; this fill only feeds the heat map.
            FloodFill();
            FloodFillElapsed = Stopwatch.GetElapsedTime(startTime);
            var layeredStart = Stopwatch.GetTimestamp();
            var layered = new LayeredSearch(CollisionMap, start, goal, startingVSpeed, DisableCactus, UseVerticalBound);
            List<Input>? inputs;
            if (LayeredUsesAStarBound)
            {
                SearchResult reference = RunNodeSearch();
                List<Input> referenceInputs = lastInputs;
                uint bound = reference.Success ? (uint)ParseFrames(Strat) : uint.MaxValue;
                inputs = bound == uint.MaxValue ? null : layered.RunWithBound(bound);
                // The sweep carries a per state minimum of the input changes, but its
                // deduplication only keeps the first arrival, so it can come out more verbose than
                // the A* it was bounded by. Same frame count, so take whichever reads shorter.
                if (inputs != null && referenceInputs.Count == inputs.Count
                    && InputChanges(referenceInputs) < InputChanges(inputs))
                {
                    inputs = referenceInputs;
                }
            }
            else
            {
                inputs = layered.Run(Math.Max(1, BeamWidth));
            }
            LayeredElapsed = Stopwatch.GetElapsedTime(layeredStart);
            SearchElapsed = LayeredElapsed;
            LayeredSimulations = layered.SimulatedUpdates;
            LayeredUpperBound = layered.UpperBound;
            LayeredBeamSucceeded = layered.BeamSucceeded;
            VisitedPlaneCount = 0;
            NodesVisited = layered.VisitedStates.ToString();
            TimeTaken = LayeredElapsed.ToString(@"dd\:hh\:mm\:ss\.ff");
            VisualizeSearch.CountStates(layered.ClosedStates);
            VisualizeSearch.HeuristicMap(GoalDistance);

            if (inputs == null)
            {
                Strat = "SEARCH FAILURE";
                return new SearchResult(Strat, "", false, layered.VisitedStates);
            }

            PointCollection points = SearchOutput.GetPathPoints(root, inputs, CollisionMap);
            PlayerPath = points;
            Macro = SearchOutput.GetMacro(inputs);
            Strat = $"Frames: {inputs.Count}\n\nVertical inputs:\n{SearchOutput.GetVerticalInputString(inputs, true)}\n\nHorizontal inputs:\n{SearchOutput.GetHorizontalInputString(inputs)}\n\nInputs per frame:\n{SearchOutput.GetInputString(inputs)}";
            var end = points.Last();
            (GoalX, GoalY) = ((int)Math.Round(end.X), (int)Math.Round(end.Y));
            return new SearchResult(Strat, Macro, true, layered.VisitedStates);
        }

        private SearchResult RunNodeSearch()
        {
            var startTime = Stopwatch.GetTimestamp();
            FloodFillElapsed = TimeSpan.Zero;
            SearchElapsed = TimeSpan.Zero;
            VisitedPlaneCount = 0;
            VisitedBitmapBytes = 0;
            VisitedOverflowCount = 0;
            PathLinkCount = 0;
            PathLinkPackedBytes = 0;
            PathLinkWideCount = 0;
            PathLinkWideBytes = 0;
            PathLinkTotalBytes = 0;
            FloodFill();
            FloodFillElapsed = Stopwatch.GetElapsedTime(startTime);
            var searchStartTime = Stopwatch.GetTimestamp();

            bool disableCactus = DisableCactus;
            int heuristicWeight = Math.Max(1, HeuristicWeight);
            uint nudgePenalty = (uint)Math.Max(0, NudgePenalty);
            // FacingRight is invisible to the physics on a map without vines, so it is
            // left out of the deduplication key and roughly halves the state space.
            bool ignoreFacing = !CollisionMap.HasVines && !PlayerNode.DisableFacingNormalization;
            PlayerNode root = new PlayerNode(start.x, start.y, startingVSpeed, RootFlags(disableCactus));

            root.PathCost = 0;
            int nodesVisited;


            bool admissible = UseAdmissibleHeuristic;
            if (admissible) admissibleTable = AdmissibleDistance.Build(CollisionMap, goal);
            ushort[]? verticalTable = UseVerticalBound ? VerticalBound.Build(CollisionMap, goal.y) : null;
            uint rootDistance = admissible ? AdmissibleDistance.At(admissibleTable!.Value, root.State.X, root.State.Y) : Distance(root);
            rootDistance = Combine(rootDistance, verticalTable, root.State);
            var openSet = new BucketQueue();
            openSet.Push(new SearchNode(root.State, root.NodeIndex), 0, rootDistance, heuristicWeight);

            var pathLinks = new PathLinkStore();
            pathLinks.ReserveRootSentinel();
            var visitedStateKeys = new VisitedStateSet();
            var neighborCandidates = new NeighborCandidate[PlayerNode.MaxNeighborCount];

            if (rootDistance != uint.MaxValue)
            {
                bool rootVisited = false;
                while (openSet.TryPop(out SearchNode v, out uint vCost))
                {
                    State vState = v.State;
                    if (v.IsGoal(goal) || CollisionMap.onWarp(v.X, v.Y))
                    {
                        SearchElapsed = Stopwatch.GetElapsedTime(searchStartTime);
                        (List<Input> inputs, PointCollection points) = SearchOutput.GetPath(root, v.NodeIndex, pathLinks, CollisionMap);
                        lastInputs = inputs;
                        TimeTaken = Stopwatch.GetElapsedTime(startTime).ToString(@"dd\:hh\:mm\:ss\.ff");
                        Macro = SearchOutput.GetMacro(inputs);
                        Strat = $"Frames: {inputs.Count}\n\nVertical inputs:\n{SearchOutput.GetVerticalInputString(inputs, true)}\n\nHorizontal inputs:\n{SearchOutput.GetHorizontalInputString(inputs)}\n\nInputs per frame:\n{SearchOutput.GetInputString(inputs)}";
                        PlayerPath = points;

                        var optimalGoal = points.Last();
                        (GoalX, GoalY) = ((int)Math.Round(optimalGoal.X), (int)Math.Round(optimalGoal.Y));
                        VisualizeSearch.CountStates(openSet, ExtractCounts());
                        VisualizeSearch.HeuristicMap(GoalDistance);
                        nodesVisited = visitedStateKeys.Count;
                        NodesVisited = nodesVisited.ToString();
                        CaptureVisitedStorage(visitedStateKeys);
                        CapturePathStorage(pathLinks);

                        return new SearchResult(Strat, macro, true, nodesVisited);
                    }
                    // Keep the original goal-at-root visited count while ensuring
                    // the root is present before any of its neighbors are checked.
                    if (!rootVisited)
                    {
                        visitedStateKeys.Add(PlayerNode.StateKey(vState, ignoreFacing));
                        rootVisited = true;
                    }

                    int currentPixelIndex = PixelIndex(v.X, v.RoundedY);
                    uint currentPathCost = vCost;
                    int neighborCount = PlayerNode.GetNeighborCandidates(vState, CollisionMap, neighborCandidates, disableCactus, ignoreFacing);
                    for (int i = 0; i < neighborCount; i++)
                    {
                        NeighborCandidate candidate = neighborCandidates[i];
                        // A state is marked discovered when it is first enqueued.
                        // Consequently the old openSet.Contains/UpdatePriority
                        // branch could never be reached for an equal state.
                        if (!visitedStateKeys.Add(candidate.Key))
                        {
                            continue;
                        }

                        // A nudge frame walks one pixel further than a normal one, so it has to
                        // earn that pixel back through NudgePenalty. The heuristic stays a lower
                        // bound on the cost because the cost can only grow.
                        uint newCost = currentPathCost + 1;
                        if (nudgePenalty > 0 && (candidate.Input & (Input.NudgeLeft | Input.NudgeRight)) != Input.Neutral)
                        {
                            newCost += nudgePenalty;
                        }
                        int roundedY = candidate.State.RoundedY;
                        int pixelIndex = PixelIndex(candidate.State.X, roundedY);
                        if (CollectHeatMap && (pixelInfo[pixelIndex] & 0xFFFF0000u) != 0xFFFF0000u)
                        {
                            pixelInfo[pixelIndex] += CounterStep;
                        }
                        uint distance = admissible
                            ? AdmissibleDistance.At(admissibleTable!.Value, candidate.State.X, candidate.State.Y)
                            : (pixelInfo[pixelIndex] & HeuristicMask) is var rawDistance && rawDistance == HeuristicUnreachable ? uint.MaxValue : rawDistance;
                        distance = Combine(distance, verticalTable, candidate.State);
                        int nodeIndex = pathLinks.Add(v.NodeIndex, candidate.Input);
                        SearchNode w = new(candidate.State, nodeIndex);
                        openSet.Push(w, newCost, distance, heuristicWeight);
                    }

                }
            }

            
            SearchElapsed = Stopwatch.GetElapsedTime(searchStartTime);
            Strat = "SEARCH FAILURE";
            VisualizeSearch.CountStates(openSet, ExtractCounts());
            VisualizeSearch.HeuristicMap(GoalDistance);
            nodesVisited = visitedStateKeys.Count;
            NodesVisited = nodesVisited.ToString();
            CaptureVisitedStorage(visitedStateKeys);
            CapturePathStorage(pathLinks);
            TimeTaken = Stopwatch.GetElapsedTime(startTime).ToString(@"hh\:mm\:ss\.ff");
            return new SearchResult(Strat, "", false, nodesVisited);
        }

        /// <summary>Unpacks the explored state counter for the heat map.</summary>
        private int[] ExtractCounts()
        {
            int[] counts = new int[Map.WIDTH * Map.HEIGHT];
            for (int i = 0; i < counts.Length; i++) counts[i] = (int)(pixelInfo[i] >> 16);
            return counts;
        }

        private static uint Combine(uint geometric, ushort[]? vertical, State state)
        {
            if (vertical == null || geometric == uint.MaxValue) return geometric;
            uint bound = VerticalBound.At(vertical, state.Y, state.VSpeed, (state.Flags & Bools.CanDJump) != Bools.None);
            return Math.Max(geometric, bound);
        }

        internal static int ParseFrames(string strat)        {
            const string prefix = "Frames: ";
            if (!strat.StartsWith(prefix)) return int.MaxValue;
            int end = strat.IndexOf('\n');
            string text = end < 0 ? strat.Substring(prefix.Length) : strat.Substring(prefix.Length, end - prefix.Length);
            return int.TryParse(text.Trim(), out int frames) ? frames : int.MaxValue;
        }

        private static ulong Priority(uint cost, uint timestamp) => ((ulong)cost << 32) | timestamp;
        private static int PixelIndex(int x, int y) => y * Map.WIDTH + x;

        /// <summary>
        /// The flags the player starts a screen with. When cactus is forbidden the jump key also starts
        /// released, so the first frames cannot release a key that was never pressed.
        /// </summary>
        private static Bools RootFlags(bool disableCactus) =>
            disableCactus ? Bools.CanDJump | Bools.FacingRight | Bools.JumpReleased : Bools.CanDJump | Bools.FacingRight;

        private void CaptureVisitedStorage(VisitedStateSet states)
        {
            VisitedPlaneCount = states.AllocatedPlaneCount;
            VisitedBitmapBytes = states.BitmapBytes;
            VisitedOverflowCount = states.OverflowCount;
        }

        private void CapturePathStorage(PathLinkStore links)
        {
            PathLinkCount = links.Count;
            PathLinkPackedBytes = links.PackedBytes;
            PathLinkWideCount = links.WideCount;
            PathLinkWideBytes = links.WideBytes;
            PathLinkTotalBytes = links.TotalBytes;
        }
    }
    public class SearchResult
    {
        public string InputString { get; } = string.Empty;
        public string Macro { get; } = string.Empty;
        public bool Success { get; }
        public int Visited { get; }

        public SearchResult(string inputString, string macro, bool success, int visited) => (InputString, Macro, Success, Visited) = (inputString, macro, success, visited);
        public override string ToString() => JsonSerializer.Serialize(this);
    }
}
