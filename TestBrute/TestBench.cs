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
            bool share, bool heat, bool optimal)
        {
            Map map = Parser.Parse(File.ReadAllText(@$"..\..\..\jmaps\{name}.jmap"));
            PlayerNode.ShareHorizontal = share;
            Search.CollectHeatMap = heat;
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
    }
}
