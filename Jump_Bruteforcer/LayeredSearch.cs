using System.Diagnostics;
using System.IO;

namespace Jump_Bruteforcer
{
    /// <summary>
    /// Layered (breadth first) search over the same transition function as <see cref="Search"/>.
    ///
    /// Two properties make it cheaper than the node-at-a-time A*:
    ///  * every edge costs one frame, so a layered sweep is optimal by construction and needs no
    ///    priority queue at all;
    ///  * states that share the exact vertical state (Y, VSpeed, Flags) also share the whole
    ///    vertical outcome of a frame, as long as neither the current nor the destination pixel
    ///    touches any geometry. Those states are grouped, one <see cref="Player.Update"/> run
    ///    serves the whole group and the horizontal offset is applied arithmetically.
    ///
    /// A cheap beam pass over the same expansion supplies an upper bound U, which the second pass
    /// uses to prune with an admissible heuristic while still guaranteeing the optimum.
    /// </summary>
    internal sealed class LayeredSearch
    {
        private const int QVCount = 189;
        private const int MinQV = -94;
        private const int QH = (Map.HEIGHT - 1) * 10 + 1;
        private const int WordsPerRow = (Map.WIDTH + 63) / 64;
        private const int VineVariants = 4;
        private const int MaxGroups = 4;
        private const int BeamHistogramSize = 4096;

        /// Value used for states whose position cannot reach the goal at all.
        private const uint FarDistance = 1u << 20;

        private readonly CollisionMap _map;
        private readonly (int x, int y) _goal;
        private readonly bool _ignoreFacing;
        private readonly bool _disableCactus;
        private readonly int _startX;
        private readonly double _startY;
        private readonly double _startVSpeed;

        private readonly ulong[][] _rowBlocked = new ulong[Map.HEIGHT][];
        private readonly ulong[][] _rowPlatform = new ulong[Map.HEIGHT][];
        private readonly ulong[][] _rowJump = new ulong[Map.HEIGHT][];
        private readonly ulong[][] _rowVine = new ulong[Map.HEIGHT][];
        private readonly AdmissibleDistance.Table _lowerBound;
        private readonly ushort[]? _vertical;
        private readonly int[] _flagToIndex = new int[256];
        private readonly Bools[] _flagOfIndex = new Bools[256];
        private int _flagCount;

        private readonly VisitedStateSet _visited = new();
        private readonly PathLinkStore _links = new();
        private readonly NeighborCandidate[] _scratch = new NeighborCandidate[PlayerNode.MaxNeighborCount];

        // Bucket heads and counting sort scratch, allocated lazily per (flags, vspeed) plane.
        private int[][] _countPlanes = Array.Empty<int[]>();
        private int[][] _startPlanes = Array.Empty<int[]>();
        private int[] _order = Array.Empty<int>();
        private readonly List<int> _touchedVKeys = new();

        // Scratch for one exact-vertical group.
        private readonly List<int> _groupMembers = new();
        private readonly double[] _groupY = new double[MaxGroups];
        private readonly double[] _groupV = new double[MaxGroups];
        private readonly Bools[] _groupFlags = new Bools[MaxGroups];
        private readonly int[] _groupMask = new int[MaxGroups];

        private int[] _beamOrder = Array.Empty<int>();
        private int[] _beamHistogram = new int[BeamHistogramSize];

        public long SimulatedUpdates { get; private set; }
        public int LayersExplored { get; private set; }
        public uint UpperBound { get; private set; } = uint.MaxValue;
        public bool BeamSucceeded { get; private set; }
        public int VisitedStates => _visited.Count;
        /// <summary>Discovered states per pixel, for the heat map.</summary>
        public int[] ClosedStates { get; } = new int[Map.WIDTH * Map.HEIGHT];
        public uint FrameCount { get; private set; }

        public LayeredSearch(CollisionMap map, (int x, double y) start, (int x, int y) goal, double startVSpeed, bool disableCactus, bool useVerticalBound = true)
        {
            _map = map;
            _goal = goal;
            _startX = start.x;
            _startY = start.y;
            _startVSpeed = startVSpeed;
            _ignoreFacing = !map.HasVines;
            _disableCactus = disableCactus;
            BuildFlagTable();
            BuildRowMasks();
            _lowerBound = AdmissibleDistance.Build(map, goal);
            _vertical = useVerticalBound ? VerticalBound.Build(map, goal.y) : null;
        }

        // ---------------------------------------------------------------- setup

        private Bools Normalize(Bools flags) => PlayerNode.NormalizeFlags(flags, _ignoreFacing);

        private void BuildFlagTable()
        {
            for (int i = 0; i < 256; i++) _flagToIndex[i] = -1;
            int next = 0;
            for (int i = 0; i < 256; i++)
            {
                Bools flags = (Bools)i;
                if ((int)Normalize(flags) != i) continue;   // only canonical combinations
                _flagToIndex[i] = next;
                _flagOfIndex[next] = flags;
                next++;
            }
            _flagCount = next;
        }

        private void BuildRowMasks()
        {
            for (int y = 0; y < Map.HEIGHT; y++)
            {
                ulong[] blocked = new ulong[WordsPerRow];
                ulong[] platform = new ulong[WordsPerRow];
                ulong[] jump = new ulong[WordsPerRow];
                ulong[] vine = new ulong[WordsPerRow];
                for (int x = 0; x < Map.WIDTH; x++)
                {
                    CollisionType type = _map.Collision[x, y];
                    ulong bit = 1UL << (x & 63);
                    int word = x >> 6;
                    if (type != CollisionType.None) blocked[word] |= bit;
                    if ((type & CollisionType.Platform) != CollisionType.None) platform[word] |= bit;
                    if ((type & (CollisionType.Solid | CollisionType.Water1 | CollisionType.Water2 | CollisionType.Water3 | CollisionType.Platform)) != CollisionType.None) jump[word] |= bit;
                    for (int variant = 0; variant < VineVariants; variant++)
                    {
                        if (_map.VineDistanceAt(x, y, variant) != VineDistance.FAR)
                        {
                            vine[x >> 6] |= 1UL << (x & 63);
                            break;
                        }
                    }
                }
                _rowBlocked[y] = blocked;
                _rowPlatform[y] = platform;
                _rowJump[y] = jump;
                _rowVine[y] = vine;
            }
        }

        // ---------------------------------------------------------------- helpers

        private static bool BitAt(ulong[] row, int x) => (row[x >> 6] & (1UL << (x & 63))) != 0;

        private uint Heuristic(int x, double y) => AdmissibleDistance.At(_lowerBound, x, y);

        /// <summary>The same bound the node search uses, for the frame pruning.</summary>
        private uint Heuristic(State state)
        {
            uint bound = Heuristic(state.X, state.Y);
            if (_vertical == null) return bound;
            uint vertical = VerticalBound.At(_vertical, state.Y, state.VSpeed, (state.Flags & Bools.CanDJump) != Bools.None);
            return Math.Max(bound, vertical);
        }

        /// <summary>
        /// True when every geometry probe that <see cref="Player.PrepareUpdateContext"/> performs at
        /// the current position misses, so the vertical outcome of the frame no longer depends on x.
        /// </summary>
        private bool ContextClean(State state)
        {
            if (state.X < 3 || state.X + 3 >= Map.WIDTH) return false;
            bool inverted = (state.Flags & Bools.InvertedGravity) != 0;
            bool kidUpsidedown = (state.Flags & Bools.ParentInvertedGravity) != 0;
            int shift = inverted ? 3 : 0;

            int vineRow = (int)Math.Round(state.Y);
            if ((uint)vineRow < Map.HEIGHT && BitAt(_rowVine[vineRow], state.X)) return false;

            int platformRow = (int)Math.Round(state.Y + (kidUpsidedown ? -4 : 4) + shift);
            if ((uint)platformRow < Map.HEIGHT && BitAt(_rowPlatform[platformRow], state.X)) return false;

            int jumpRow = (int)Math.Round(state.Y + (inverted ? -1 : 1) + shift);
            if ((uint)jumpRow < Map.HEIGHT && BitAt(_rowJump[jumpRow], state.X)) return false;

            return true;
        }

        /// <summary>Every pixel a frame from this x could land on has to be free.</summary>
        private bool DestinationsFree(int x, bool inverted)
        {
            for (int group = 0; group < MaxGroups; group++)
            {
                if (_groupMask[group] == 0) continue;
                int row = (int)Math.Round(_groupY[group] + (inverted ? 3 : 0));
                if ((uint)row >= Map.HEIGHT) return false;
                ulong[] blocked = _rowBlocked[row];
                if (BitAt(blocked, x - 3) || BitAt(blocked, x) || BitAt(blocked, x + 3)) return false;
            }
            return true;
        }

        private ulong KeyOf(State state) => PlayerNode.StateKey(state, _ignoreFacing);

        private int BucketOf(State state)
        {
            int quantizedY = Math.Clamp((int)Math.Round(state.Y * 10), 0, QH - 1);
            int quantizedVSpeed = Math.Clamp((int)Math.Round(state.VSpeed * 10), MinQV, MinQV + QVCount - 1);
            int flagIndex = _flagToIndex[(int)Normalize(state.Flags)];
            if (flagIndex < 0) flagIndex = 0;
            return (flagIndex * QVCount + (quantizedVSpeed - MinQV)) * QH + quantizedY;
        }

        private int PlaneOf(int bucket) => bucket / QH;

        private int RowOf(int bucket) => bucket % QH;

        private bool IsGoal(State state) =>
            (Math.Abs(state.X - _goal.x) <= 1 && state.RoundedY == _goal.y) || _map.onWarp(state.X, state.Y);

        // ---------------------------------------------------------------- driver

        /// <summary>
        /// Runs only the bounded optimal pass with a bound supplied from the outside (typically the
        /// frame count of an A* solution). The beam pass is skipped entirely.
        /// </summary>
        public List<Input>? RunWithBound(uint bound)
        {
            SimulatedUpdates = 0;
            LayersExplored = 0;
            UpperBound = bound;
            BeamSucceeded = false;
            FrameCount = 0;
            ResetPass();
            List<Input>? optimal = RunPass(0, bound, prune: true);
            if (optimal != null) FrameCount = (uint)optimal.Count;
            return optimal;
        }

        /// <summary>Runs the beam pass and then the optimal pass. Returns null when unsolvable.</summary>
        public List<Input>? Run(int beamWidth)
        {
            SimulatedUpdates = 0;
            LayersExplored = 0;
            UpperBound = uint.MaxValue;
            BeamSucceeded = false;
            FrameCount = 0;

            List<Input>? beamPath = null;
            for (int attempt = 0; attempt < 3 && beamPath == null; attempt++)
            {
                ResetPass();
                beamPath = RunPass(beamWidth << (2 * attempt), uint.MaxValue, prune: false);
                if (beamPath != null)
                {
                    BeamSucceeded = true;
                    UpperBound = (uint)beamPath.Count;
                }
            }

            ResetPass();
            List<Input>? optimal = RunPass(0, UpperBound, prune: true);
            if (optimal != null)
            {
                FrameCount = (uint)optimal.Count;
                return optimal;
            }
            if (beamPath != null) FrameCount = (uint)beamPath.Count;
            return beamPath;
        }

        private void ResetPass()
        {
            _visited.Clear();
            _links.Clear();
            _links.ReserveRootSentinel();
            _countPlanes = new int[_flagCount * QVCount][];
            _startPlanes = new int[_flagCount * QVCount][];
            _touchedVKeys.Clear();
        }

        private List<Input>? RunPass(int beamWidth, uint frameBound, bool prune)
        {
            State root = new()
            {
                X = _startX,
                Y = _startY,
                VSpeed = _startVSpeed,
                Flags = _disableCactus
                    ? Bools.CanDJump | Bools.FacingRight | Bools.JumpReleased
                    : Bools.CanDJump | Bools.FacingRight,
            };

            if (IsGoal(root)) return new List<Input>();

            var cur = new Frontier();
            var next = new Frontier();
            cur.Add(root.X, root.Y, root.VSpeed, (byte)root.Flags, 0, BucketOf(root));
            _visited.Add(KeyOf(root));

            for (int layer = 0; layer < Map.WIDTH * Map.HEIGHT; layer++)
            {
                if (cur.Count == 0) return null;
                int goalIndex = ExpandLayer(cur, next, layer, prune ? frameBound : uint.MaxValue, beamWidth);
                LayersExplored = layer + 1;
                if (goalIndex >= 0) return Reconstruct(goalIndex);
                (cur, next) = (next, cur);
                next.Clear();
                if (beamWidth == 0 && frameBound != uint.MaxValue && layer + 1 > frameBound) return null;
            }
            return null;
        }

        private List<Input> Reconstruct(int goalIndex)
        {
            var inputs = new List<Input>();
            int index = goalIndex;
            while (index != 0)
            {
                PathLink link = _links[index];
                inputs.Add(link.Input);
                index = link.ParentIndex;
            }
            inputs.Reverse();
            return inputs;
        }

        // ---------------------------------------------------------------- expansion

        private int ExpandLayer(Frontier cur, Frontier next, int layer, uint frameBound, int beamWidth)
        {
            BuildOrder(cur);

            for (int t = 0; t < _touchedVKeys.Count; t++)
            {
                int bucket = _touchedVKeys[t];
                int plane = PlaneOf(bucket), row = RowOf(bucket);
                // The scatter pass walks the start cursor down to the bucket begin.
                int begin = _startPlanes[plane][row];
                int count = _countPlanes[plane][row];
                int end = begin + count;
                if (count == 0) continue;

                for (int slot = begin; slot < end; slot++)
                {
                    int i = _order[slot];
                    if (cur.Processed[i]) continue;

                    CollectGroup(cur, begin, end, i);
                    State representative = cur.StateAt(i);
                    int emitted = 0;
                    if (ContextClean(representative) && GroupFits(cur, representative))
                    {
                        emitted = EmitGrouped(cur, next, layer, frameBound, next);
                        if (emitted > 0)
                        {
                            foreach (int member in _groupMembers) cur.Processed[member] = true;
                        }
                    }
                    if (emitted == 0)
                    {
                        foreach (int member in _groupMembers)
                        {
                            EmitDirect(cur, next, layer, frameBound, member);
                            cur.Processed[member] = true;
                        }
                    }

                    if (next.GoalIndex >= 0) return next.GoalIndex;
                }
            }

            ClearOrder();
            if (beamWidth > 0) TrimToBeam(next, beamWidth);
            return -1;
        }

        /// <summary>Groups the bucket members that share the exact vertical state.</summary>
        private void CollectGroup(Frontier cur, int begin, int end, int seed)
        {
            _groupMembers.Clear();
            _groupMembers.Add(seed);
            for (int slot = begin; slot < end; slot++)
            {
                int j = _order[slot];
                if (j == seed || cur.Processed[j]) continue;
                if (cur.Y[j] != cur.Y[seed] || cur.V[j] != cur.V[seed]) continue;
                if (Normalize((Bools)cur.Flags[j]) != Normalize((Bools)cur.Flags[seed])) continue;
                _groupMembers.Add(j);
            }
        }

        /// <summary>
        /// Runs the real neighbour generator once for the group representative; because the whole
        /// group shares the exact vertical state and a clean context, its results apply to every
        /// member after shifting x.
        /// </summary>
        private bool GroupFits(Frontier cur, State representative)
        {
            Array.Clear(_groupMask, 0, MaxGroups);
            int n = PlayerNode.GetNeighborCandidates(representative, _map, _scratch, _disableCactus, _ignoreFacing);
            SimulatedUpdates += n;
            for (int i = 0; i < n; i++)
            {
                NeighborCandidate candidate = _scratch[i];
                if (candidate.State.X != representative.X) return false;   // only a clean frame shifts like this
                int group = ((candidate.Input & Input.Jump) != 0 ? 1 : 0) | ((candidate.Input & Input.Release) != 0 ? 2 : 0);
                int horizontal = (candidate.Input & Input.Left) != 0 ? 0 : (candidate.Input & Input.Right) != 0 ? 2 : 1;
                if (_groupMask[group] == 0)
                {
                    _groupY[group] = candidate.State.Y;
                    _groupV[group] = candidate.State.VSpeed;
                    _groupFlags[group] = candidate.State.Flags;
                }
                _groupMask[group] |= 1 << horizontal;
            }
            for (int group = 0; group < MaxGroups; group++)
            {
                if (_groupMask[group] == 0) continue;
                if (_groupMask[group] != 0b111) return false;
            }

            return true;
        }

        /// <summary>One destination pixel; only then can the frame be shared.</summary>
        private bool ShiftFree(int x, double y, bool inverted)
        {
            if ((uint)x >= Map.WIDTH) return false;
            int row = (int)Math.Round(y + (inverted ? 3 : 0));
            if ((uint)row >= Map.HEIGHT) return false;
            return !BitAt(_rowBlocked[row], x);
        }

        private int EmitGrouped(Frontier cur, Frontier next, int layer, uint frameBound, Frontier _)
        {
            int emitted = 0;
            foreach (int member in _groupMembers)
            {
                State state = cur.StateAt(member);
                bool clean = ContextClean(state);
                bool inverted = (state.Flags & Bools.InvertedGravity) != 0;
                for (int group = 0; group < MaxGroups; group++)
                {
                    if (_groupMask[group] == 0) continue;
                    bool jump = (group & 1) != 0;
                    bool release = (group & 2) != 0;
                    for (int horizontal = 0; horizontal < 3; horizontal++)
                    {
                        int shift = horizontal == 0 ? -1 : horizontal == 2 ? 1 : 0;
                        Input input = Input.Neutral;
                        if (jump) input |= Input.Jump;
                        if (release) input |= Input.Release;
                        if (shift < 0) input |= Input.Left;
                        else if (shift > 0) input |= Input.Right;

                        Bools flags = _groupFlags[group];
                        if (shift > 0) flags |= Bools.FacingRight;
                        else if (shift < 0) flags &= ~Bools.FacingRight;

                        State target;
                        int shiftedX = state.X + shift * PhysicsParams.WALKING_SPEED;
                        if (clean && ShiftFree(shiftedX, _groupY[group], inverted))
                        {
                            target = new State
                            {
                                X = shiftedX,
                                Y = _groupY[group],
                                VSpeed = _groupV[group],
                                Flags = flags,
                            };
                        }
                        else
                        {
                            State? resolved = Player.Update(state, input, _map);
                            SimulatedUpdates++;
                            if (resolved is not State full) continue;
                            target = full;
                        }
                        emitted++;
                        if (Add(next, target, input, cur.Idx[member], layer, frameBound)) return emitted;
                    }
                }
            }
            return emitted;
        }
        /// <summary>Fully simulates one state, exactly like the node based search does.</summary>
        private void EmitDirect(Frontier cur, Frontier next, int layer, uint frameBound, int index)
        {
            State state = cur.StateAt(index);
            int n = PlayerNode.GetNeighborCandidates(state, _map, _scratch, _disableCactus, _ignoreFacing);
            SimulatedUpdates += n;
            for (int i = 0; i < n; i++)
            {
                Add(next, _scratch[i].State, _scratch[i].Input, cur.Idx[index], layer, frameBound);
                if (next.GoalIndex >= 0) return;
            }
        }

        /// <summary>Deduplicates, prunes and appends a successor. Returns true when it is the goal.</summary>
        private bool Add(Frontier next, State state, Input input, int parentIndex, int layer, uint frameBound)
        {
            if (!InBounds(state.X, state.Y)) return false;
            ulong key = KeyOf(state);
            if (!_visited.Add(key)) return false;
            if (frameBound != uint.MaxValue && (ulong)(uint)(layer + 1) + Heuristic(state.X, state.Y) > frameBound)
            {
                return false;
            }
            int nodeIndex = _links.Add(parentIndex, input);
            ClosedStates[state.RoundedY * Map.WIDTH + state.X] += 1;
            next.Add(state.X, state.Y, state.VSpeed, (byte)state.Flags, nodeIndex, BucketOf(state));
            if (IsGoal(state))
            {
                // The goal index has to be the path link, not the position inside the layer.
                next.GoalIndex = nodeIndex;
                return true;
            }
            return false;
        }

        private static bool InBounds(int x, double y) =>
            x >= 0 && x < Map.WIDTH && y >= 0 && y <= Map.HEIGHT - 1;

        // ---------------------------------------------------------------- counting sort

        private void BuildOrder(Frontier cur)
        {
            if (_countPlanes.Length == 0) ResetPass();
            _touchedVKeys.Clear();
            if (_order.Length < cur.Count) _order = new int[Math.Max(cur.Count, 1024)];
            cur.EnsureProcessed();

            for (int i = 0; i < cur.Count; i++)
            {
                int bucket = cur.VKey[i];
                int plane = PlaneOf(bucket), row = RowOf(bucket);
                if (_countPlanes[plane] == null)
                {
                    _countPlanes[plane] = new int[QH];
                    _startPlanes[plane] = new int[QH];
                }
                if (_countPlanes[plane][row]++ == 0) _touchedVKeys.Add(bucket);
            }

            int offset = 0;
            foreach (int bucket in _touchedVKeys)
            {
                int plane = PlaneOf(bucket), row = RowOf(bucket);
                _startPlanes[plane][row] = offset + _countPlanes[plane][row];
                offset += _countPlanes[plane][row];
            }

            for (int i = 0; i < cur.Count; i++)
            {
                int bucket = cur.VKey[i];
                int plane = PlaneOf(bucket), row = RowOf(bucket);
                _order[--_startPlanes[plane][row]] = i;
            }
        }

        private void ClearOrder()
        {
            foreach (int bucket in _touchedVKeys)
            {
                int plane = PlaneOf(bucket), row = RowOf(bucket);
                _countPlanes[plane][row] = 0;
                _startPlanes[plane][row] = 0;
            }
            _touchedVKeys.Clear();
        }

        // ---------------------------------------------------------------- beam

        private void TrimToBeam(Frontier frontier, int width)
        {
            int count = frontier.Count;
            if (count <= width) return;
            if (_beamOrder.Length < count) _beamOrder = new int[count];
            Array.Clear(_beamHistogram, 0, BeamHistogramSize);

            for (int i = 0; i < count; i++)
            {
                uint h = Heuristic(frontier.X[i], frontier.Y[i]);
                int bucket = (int)Math.Min(h, BeamHistogramSize - 1);
                _beamOrder[i] = bucket;
                _beamHistogram[bucket]++;
            }

            int offset = 0;
            for (int b = 0; b < BeamHistogramSize; b++)
            {
                int n = _beamHistogram[b];
                _beamHistogram[b] = offset;
                offset += n;
            }

            var sorted = new int[count];
            for (int i = 0; i < count; i++)
            {
                sorted[_beamHistogram[_beamOrder[i]]++] = i;
            }

            var trimmed = new Frontier();
            int keep = Math.Min(width, count);
            for (int i = 0; i < keep; i++)
            {
                int source = sorted[i];
                trimmed.Add(frontier.X[source], frontier.Y[source], frontier.V[source],
                    frontier.Flags[source], frontier.Idx[source], frontier.VKey[source]);
            }
            frontier.CopyFrom(trimmed);
        }

        // ---------------------------------------------------------------- storage

        private sealed class Frontier
        {
            public int[] X = new int[1024];
            public double[] Y = new double[1024];
            public double[] V = new double[1024];
            public byte[] Flags = new byte[1024];
            public int[] Idx = new int[1024];
            public int[] VKey = new int[1024];
            public bool[] Processed = new bool[1024];
            public int Count;
            public int GoalIndex = -1;

            public void EnsureProcessed()
            {
                if (Processed.Length < X.Length) Processed = new bool[X.Length];
                Array.Clear(Processed, 0, Count);
            }

            public State StateAt(int i) => new() { X = X[i], Y = Y[i], VSpeed = V[i], Flags = (Bools)Flags[i] };

            public int Add(int x, double y, double v, byte flags, int index, int vkey)
            {
                if (Count == X.Length) Grow();
                X[Count] = x;
                Y[Count] = y;
                V[Count] = v;
                Flags[Count] = flags;
                Idx[Count] = index;
                VKey[Count] = vkey;
                Count++;
                return Count - 1;
            }

            public void CopyFrom(Frontier other)
            {
                if (X.Length < other.Count) Grow(other.Count);
                Array.Copy(other.X, X, other.Count);
                Array.Copy(other.Y, Y, other.Count);
                Array.Copy(other.V, V, other.Count);
                Array.Copy(other.Flags, Flags, other.Count);
                Array.Copy(other.Idx, Idx, other.Count);
                Array.Copy(other.VKey, VKey, other.Count);
                Count = other.Count;
                GoalIndex = other.GoalIndex;
            }

            public void Clear()
            {
                Count = 0;
                GoalIndex = -1;
            }

            private void Grow(int needed = 0)
            {
                int capacity = Math.Max(X.Length * 2, Math.Max(needed, 1024));
                Array.Resize(ref X, capacity);
                Array.Resize(ref Y, capacity);
                Array.Resize(ref V, capacity);
                Array.Resize(ref Flags, capacity);
                Array.Resize(ref Idx, capacity);
                Array.Resize(ref VKey, capacity);
                Array.Resize(ref Processed, capacity);
            }
        }
    }
}
