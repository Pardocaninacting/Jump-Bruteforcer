using Jump_Bruteforcer;
using System.Windows;
using Xunit.Abstractions;

namespace TestBrute
{
    /// <summary>
    /// Benchmark harness for the search optimizations. Emits machine readable CSV lines:
    /// config,map,success,frames,visited,milliseconds,visitedBitmapBytes,pathLinkBytes,managedBytes.
    /// Run with: dotnet test --filter FullyQualifiedName~TestBench
    /// Exclude from the normal suite with: dotnet test --filter "Category!=Bench"
    /// </summary>
    public class TestBench
    {
        private readonly ITestOutputHelper output;

        public TestBench(ITestOutputHelper output)
        {
            if (Application.Current == null) new Application();
            this.output = output;
        }

        public static (int x, double y, int gx, int gy, string name, bool big)[] Cases =
        {
            (753, 567.4, 743, 119, "nameless", true),
            (49, 567, 771, 231, "i_wanna_x", true),
            (241, 119.4, 541, 231, "decession", true),
            (410, 407.4, 476, 343, "tomo", false),
            (452, 407.4, 482, 343, "minif", false),
            (420, 407.4, 477, 375, "ground_dplane", false),
            (410, 407.4, 485, 407, "co", false),
            (388, 407.4, 541, 407, "platform_invert", false),
        };

        private void Run(string config, int sx, double sy, int gx, int gy, string name,
            bool share, bool heat, bool optimal, bool nudge = false)
        {
            Map map = Parser.Parse(File.ReadAllText(@$"..\..\..\jmaps\{name}.jmap"));
            PlayerNode.ShareHorizontal = share;
            Search.CollectHeatMap = heat;
            Player.AllowNudge = nudge;
            Search s = new((sx, sy), (gx, gy), map.CollisionMap)
            {
                UseLayeredBfs = optimal,
                LayeredUsesAStarBound = true,
            };
            var sw = System.Diagnostics.Stopwatch.StartNew();
            SearchResult r = s.RunAStar();
            sw.Stop();
            output.WriteLine($"CSV,{config},{name},{r.Success},{s.Strat.Split('\n')[0].Replace("Frames: ", "")}," +
                             $"{s.NodesVisited},{sw.ElapsedMilliseconds},{s.VisitedBitmapBytes},{s.PathLinkTotalBytes}," +
                             $"{GC.GetTotalMemory(false)}");
            PlayerNode.ShareHorizontal = true;
            Search.CollectHeatMap = true;
            Player.AllowNudge = false;
        }

        [Fact]
        [Trait("Category", "Bench")]
        public void BenchSmallMaps()
        {
            output.WriteLine("CSV,config,map,success,frames,visited,ms,planeBytes,linkBytes,managedBytes");
            foreach (var c in Cases)
            {
                if (c.big) continue;
                Run("default", c.x, c.y, c.gx, c.gy, c.name, true, true, false);
                Run("noshare", c.x, c.y, c.gx, c.gy, c.name, false, true, false);
                Run("noheat", c.x, c.y, c.gx, c.gy, c.name, true, false, false);
                Run("optimal", c.x, c.y, c.gx, c.gy, c.name, true, true, true);
            }
        }

        [Fact]
        [Trait("Category", "Bench")]
        public void BenchBigMaps()
        {
            foreach (var c in Cases)
            {
                if (!c.big) continue;
                Run("default", c.x, c.y, c.gx, c.gy, c.name, true, true, false);
                Run("noshare", c.x, c.y, c.gx, c.gy, c.name, false, true, false);
                Run("noheat", c.x, c.y, c.gx, c.gy, c.name, true, false, false);
                Run("optimal", c.x, c.y, c.gx, c.gy, c.name, true, true, true);
            }
        }

        [Fact]
        [Trait("Category", "Bench")]
        public void BenchAblation()
        {
            foreach (var c in Cases)
            {
                if (!c.big) continue;
                for (int rep = 0; rep < 2; rep++)
                {
                    Run($"a{rep}-default", c.x, c.y, c.gx, c.gy, c.name, true, true, false);
                    Run($"a{rep}-noshare", c.x, c.y, c.gx, c.gy, c.name, false, true, false);
                    Run($"a{rep}-noheat", c.x, c.y, c.gx, c.gy, c.name, true, false, false);
                    PlayerNode.DisableFacingNormalization = true;
                    Run($"a{rep}-nonorm", c.x, c.y, c.gx, c.gy, c.name, true, true, false);
                    PlayerNode.DisableFacingNormalization = false;
                }
            }
        }

        [Fact]
        [Trait("Category", "Bench")]
        public void BenchDefault()
        {
            foreach (var c in Cases)
            {
                if (!c.big) continue;
                Run("default", c.x, c.y, c.gx, c.gy, c.name, true, true, false);
            }
        }

        [Fact]
        [Trait("Category", "Bench")]
        public void BenchRepeat()
        {
            // Interleaved repetitions: controls for JIT warm up, thermal drift and run order.
            foreach (var c in Cases)
            {
                if (!c.big) continue;
                for (int rep = 0; rep < 3; rep++)
                {
                    Run($"r{rep}-share", c.x, c.y, c.gx, c.gy, c.name, true, true, false);
                    Run($"r{rep}-noshare", c.x, c.y, c.gx, c.gy, c.name, false, true, false);
                }
            }
        }

        /// <summary>Maps with a lot of ground time, where the nudge adds the most neighbours.</summary>
        public static (int x, double y, int gx, int gy, string name)[] NudgeCases =
        {
            (452, 407.4, 482, 343, "minif"),
            (420, 407.4, 477, 375, "ground_dplane"),
            (410, 407.4, 485, 407, "co"),
            (388, 407.4, 541, 407, "platform_invert"),
            (399, 487.4, 399, 295, "platform_teleport"),
            (399, 487.4, 399, 295, "platform_elevator"),
            (401, 407.4, 602, 407, "dt"),          // only solvable with the nudge
        };

        [Fact]
        [Trait("Category", "Bench")]
        public void BenchNudge()
        {
            // Interleaved on/off repetitions, min of N: the option is a global, so the two
            // configurations have to be flipped around each run.
            output.WriteLine("CSV,config,map,success,frames,visited,ms,planeBytes,linkBytes,managedBytes");
            foreach (var c in NudgeCases)
            {
                for (int rep = 0; rep < 3; rep++)
                {
                    Run($"n{rep}-off", c.x, c.y, c.gx, c.gy, c.name, true, true, false, nudge: false);
                    Run($"n{rep}-on", c.x, c.y, c.gx, c.gy, c.name, true, true, false, nudge: true);
                }
            }
        }

        [Fact]
        [Trait("Category", "Bench")]
        public void BenchNudgeBig()
        {
            output.WriteLine("CSV,config,map,success,frames,visited,ms,planeBytes,linkBytes,managedBytes");
            foreach (var c in new[] { (753, 183, 217, 220, "nabla_2"), (49, 567, 771, 231, "i_wanna_x") })
            {
                for (int rep = 0; rep < 2; rep++)
                {
                    Run($"b{rep}-off", c.Item1, c.Item2, c.Item3, c.Item4, c.Item5, true, true, false, nudge: false);
                    Run($"b{rep}-on", c.Item1, c.Item2, c.Item3, c.Item4, c.Item5, true, true, false, nudge: true);
                }
            }
        }
    }
}
