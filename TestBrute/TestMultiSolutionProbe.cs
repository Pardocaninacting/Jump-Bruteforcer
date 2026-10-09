using Jump_Bruteforcer;
using System.Numerics;
using System.Windows;
using Xunit.Abstractions;

namespace TestBrute
{
    /// <summary>
    /// How many frame-optimal solutions does a screen actually have? Rebuilds the state graph
    /// layer by layer from the public neighbour generator and counts the paths that reach the goal
    /// on the first layer that reaches it at all, then samples some of them to see how different
    /// they are. Nothing is deduplicated between layers, so the graph is the full reachable set
    /// and it can be much larger than what the search itself visits; screens whose layer grows past
    /// MaxLayerStates are reported as aborted. Analysis tool, not a regression test:
    /// run with --filter FullyQualifiedName~TestMultiSolutionProbe.
    /// </summary>
    public class TestMultiSolutionProbe
    {
        private readonly ITestOutputHelper output;

        public TestMultiSolutionProbe(ITestOutputHelper output)
        {
            if (Application.Current == null) new Application();
            this.output = output;
        }

        public static (int x, double y, int gx, int gy, string name)[] Cases =
        {
            (420, 407.4, 477, 375, "ground_dplane"),
            (399, 487.4, 399, 295, "platform_teleport"),
            (399, 487.4, 399, 295, "platform_elevator"),
            (410, 407.4, 485, 407, "co"),
            (388, 407.4, 541, 407, "platform_invert"),
            (452, 407.4, 482, 343, "minif"),
        };

        // The largest layer in Cases is platform_teleport with the nudge on: 2,988,515 states.
        // Each one costs roughly 200 bytes here (state, path count and predecessor list), so the
        // biggest case needs around 600 MB.
        private const int MaxLayerStates = 3_000_000;
        private const int Samples = 200;

        [Fact]
        [Trait("Category", "Bench")]
        public void ProbeSolutionCounts()
        {
            output.WriteLine("map,nudge,astarFrames,optimalFrames,layeredFrames,goalStates,paths,distinctInputs,distinctTracks,inputDiff,trackDiff,validSamples,maxLayerStates,seconds");
            foreach (var c in Cases)
            {
                Run(c, allowNudge: false);
            }
        }

        [Fact]
        [Trait("Category", "Bench")]
        public void ProbeSolutionCountsWithNudge()
        {
            output.WriteLine("map,nudge,astarFrames,optimalFrames,layeredFrames,goalStates,paths,distinctInputs,distinctTracks,inputDiff,trackDiff,validSamples,maxLayerStates,seconds");
            foreach (var c in Cases)
            {
                Run(c, allowNudge: true);
            }
        }

        private void Run((int x, double y, int gx, int gy, string name) c, bool allowNudge)
        {
            Map map = Parser.Parse(File.ReadAllText(@$"..\..\..\jmaps\{c.name}.jmap"));
            Player.AllowNudge = allowNudge;
            var sw = System.Diagnostics.Stopwatch.StartNew();

            // what the tool's own two searches report for the same screen
            Search astar = new((c.x, c.y), (c.gx, c.gy), map.CollisionMap) { AllowNudge = allowNudge };
            SearchResult astarResult = astar.RunAStar();
            Search layered = new((c.x, c.y), (c.gx, c.gy), map.CollisionMap) { AllowNudge = allowNudge, UseLayeredBfs = true };
            SearchResult layeredResult = layered.RunAStar();
            Player.AllowNudge = allowNudge;
            int astarFrames = astarResult.Success ? Frames(astarResult) : -1;
            int layeredFrames = layeredResult.Success ? Frames(layeredResult) : -1;

            bool ignoreFacing = !map.CollisionMap.HasVines && !PlayerNode.DisableFacingNormalization;
            var root = new PlayerNode(c.x, c.y, 0);
            ulong rootKey = PlayerNode.StateKey(root.State, ignoreFacing);

            // layer 0
            var states = new List<Dictionary<ulong, State>> { new() { [rootKey] = root.State } };
            var ways = new List<Dictionary<ulong, BigInteger>> { new() { [rootKey] = BigInteger.One } };
            var preds = new List<Dictionary<ulong, List<(ulong Prev, Input Input)>>> { new() { [rootKey] = new() } };

            int frames = -1;
            BigInteger paths = BigInteger.Zero;
            int goalStates = 0;
            int maxLayer = 1;

            for (int layer = 1; layer <= 400; layer++)
            {
                var nextStates = new Dictionary<ulong, State>();
                var nextWays = new Dictionary<ulong, BigInteger>();
                var nextPreds = new Dictionary<ulong, List<(ulong Prev, Input Input)>>();
                var buffer = new NeighborCandidate[PlayerNode.MaxNeighborCount];

                foreach (var (key, state) in states[layer - 1])
                {
                    BigInteger waysHere = ways[layer - 1][key];
                    int n = PlayerNode.GetNeighborCandidates(state, map.CollisionMap, buffer, false, ignoreFacing);
                    for (int i = 0; i < n; i++)
                    {
                        NeighborCandidate candidate = buffer[i];
                        if (nextWays.TryGetValue(candidate.Key, out BigInteger existing))
                        {
                            nextWays[candidate.Key] = existing + waysHere;
                        }
                        else
                        {
                            nextStates[candidate.Key] = candidate.State;
                            nextWays[candidate.Key] = waysHere;
                        }

                        if (!nextPreds.TryGetValue(candidate.Key, out var list))
                        {
                            list = new List<(ulong, Input)>();
                            nextPreds[candidate.Key] = list;
                        }
                        list.Add((key, candidate.Input));
                    }
                }

                states.Add(nextStates);
                ways.Add(nextWays);
                preds.Add(nextPreds);
                maxLayer = Math.Max(maxLayer, nextStates.Count);

                foreach (var (key, state) in nextStates)
                {
                    if (Math.Abs(state.X - c.gx) <= 1 && state.RoundedY == c.gy)
                    {
                        paths += nextWays[key];
                        goalStates++;
                    }
                }

                if (goalStates > 0)
                {
                    frames = layer;
                    break;
                }
                if (nextStates.Count == 0 || nextStates.Count > MaxLayerStates)
                {
                    break;
                }
            }

            if (frames < 0)
            {
                output.WriteLine($"{c.name},{allowNudge},{astarFrames},aborted,{layeredFrames},-,-,-,-,-,-,-,{maxLayer},{sw.Elapsed.TotalSeconds:0.0}");
                Player.AllowNudge = false;
                return;
            }

            (int distinctInputs, int distinctTracks, double inputDiff, double trackDiff, int valid) =
                Sample(states, ways, preds, frames, c, map, ignoreFacing);
            output.WriteLine($"{c.name},{allowNudge},{astarFrames},{frames},{layeredFrames},{goalStates},{paths}," +
                             $"{distinctInputs},{distinctTracks},{inputDiff:0.0},{trackDiff:0.0},{valid},{maxLayer},{sw.Elapsed.TotalSeconds:0.0}");
            Player.AllowNudge = false;
        }

        private static int Frames(SearchResult result) =>
            int.Parse(System.Text.RegularExpressions.Regex.Match(result.InputString, @"Frames: (\d+)").Groups[1].Value);

        /// <summary>
        /// Walks back from a goal, picking predecessors in proportion to their path count, and
        /// compares the samples both as input sequences and as the positions they visit.
        /// </summary>
        private (int DistinctInputs, int DistinctTracks, double InputDiff, double TrackDiff, int Valid) Sample(
            List<Dictionary<ulong, State>> states, List<Dictionary<ulong, BigInteger>> ways,
            List<Dictionary<ulong, List<(ulong Prev, Input Input)>>> preds, int frames,
            (int x, double y, int gx, int gy, string name) c, Map map, bool ignoreFacing)
        {
            var goals = new List<(ulong Key, BigInteger Ways)>();
            foreach (var (key, state) in states[frames])
            {
                if (Math.Abs(state.X - c.gx) <= 1 && state.RoundedY == c.gy)
                {
                    goals.Add((key, ways[frames][key]));
                }
            }
            if (goals.Count == 0)
            {
                return (0, 0, 0, 0, 0);
            }

            var rng = new Random(12345);
            var samples = new List<Input[]>();
            var tracks = new List<string>();
            int valid = 0;
            for (int s = 0; s < Samples; s++)
            {
                ulong cur = Pick(rng, goals);
                var inputs = new Input[frames];
                for (int layer = frames; layer >= 1; layer--)
                {
                    var candidates = new List<(ulong Prev, Input Input, BigInteger Ways)>();
                    foreach (var (prev, input) in preds[layer][cur])
                    {
                        candidates.Add((prev, input, ways[layer - 1][prev]));
                    }
                    BigInteger total = BigInteger.Zero;
                    foreach (var candidate in candidates)
                    {
                        total += candidate.Ways;
                    }
                    BigInteger r = RandomBelow(rng, total);
                    foreach (var candidate in candidates)
                    {
                        if (r < candidate.Ways)
                        {
                            inputs[layer - 1] = candidate.Input;
                            cur = candidate.Prev;
                            break;
                        }
                        r -= candidate.Ways;
                    }
                }
                samples.Add(inputs);
                string track = Track(new PlayerNode(c.x, c.y, 0), inputs, map);
                tracks.Add(track);
                if (track.EndsWith($"{c.gx}:{c.gy};") || EndsNear(track, c.gx, c.gy))
                {
                    valid++;
                }
            }

            int distinctInputs = samples.Select(Signature).Distinct().Count();
            int distinctTracks = tracks.Distinct().Count();
            return (distinctInputs, distinctTracks,
                    MeanInputDiff(samples, frames),
                    MeanTrackDiff(tracks),
                    valid);
        }

        /// <summary>True when the replayed end position is inside the goal region.</summary>
        private static bool EndsNear(string track, int gx, int gy)
        {
            string[] frames = track.Split(';', StringSplitOptions.RemoveEmptyEntries);
            string[] last = frames[^1].Split(':');
            return Math.Abs(int.Parse(last[0]) - gx) <= 1 && int.Parse(last[1]) == gy;
        }

        /// <summary>Positions the kid visits, one "x:y" per frame.</summary>
        private static string Track(PlayerNode root, Input[] inputs, Map map)
        {
            var sb = new System.Text.StringBuilder(inputs.Length * 8);
            PlayerNode cur = root;
            sb.Append(cur.State.X).Append(':').Append(cur.State.RoundedY).Append(';');
            foreach (Input input in inputs)
            {
                PlayerNode? next = cur.NewState(input, map.CollisionMap);
                if (next is null)
                {
                    break;
                }
                cur = next;
                sb.Append(cur.State.X).Append(':').Append(cur.State.RoundedY).Append(';');
            }
            return sb.ToString();
        }

        private static double MeanInputDiff(List<Input[]> samples, int frames)
        {
            double total = 0;
            long pairs = 0;
            for (int i = 1; i < samples.Count; i++)
            {
                for (int j = 0; j < i; j++)
                {
                    int diff = 0;
                    for (int f = 0; f < frames; f++)
                    {
                        if (samples[i][f] != samples[j][f])
                        {
                            diff++;
                        }
                    }
                    total += diff;
                    pairs++;
                }
            }
            return pairs == 0 ? 0 : total / pairs;
        }

        private static double MeanTrackDiff(List<string> tracks)
        {
            string[][] split = tracks.Select(t => t.Split(';', StringSplitOptions.RemoveEmptyEntries)).ToArray();
            double total = 0;
            long pairs = 0;
            for (int i = 1; i < split.Length; i++)
            {
                for (int j = 0; j < i; j++)
                {
                    int len = Math.Min(split[i].Length, split[j].Length);
                    int diff = 0;
                    for (int f = 0; f < len; f++)
                    {
                        if (split[i][f] != split[j][f])
                        {
                            diff++;
                        }
                    }
                    total += diff;
                    pairs++;
                }
            }
            return pairs == 0 ? 0 : total / pairs;
        }

        private static string Signature(Input[] inputs) => string.Join(",", inputs.Select(i => (byte)i));

        private static ulong Pick(Random rng, List<(ulong Key, BigInteger Ways)> entries)
        {
            BigInteger total = BigInteger.Zero;
            foreach (var entry in entries)
            {
                total += entry.Ways;
            }
            BigInteger r = RandomBelow(rng, total);
            foreach (var entry in entries)
            {
                if (r < entry.Ways)
                {
                    return entry.Key;
                }
                r -= entry.Ways;
            }
            return entries[^1].Key;
        }

        private static BigInteger RandomBelow(Random rng, BigInteger limit)
        {
            if (limit <= 1)
            {
                return BigInteger.Zero;
            }
            int bytes = limit.GetByteCount();
            var buffer = new byte[bytes + 1];
            rng.NextBytes(buffer);
            buffer[bytes] = 0;
            return new BigInteger(buffer) % limit;
        }
    }
}
