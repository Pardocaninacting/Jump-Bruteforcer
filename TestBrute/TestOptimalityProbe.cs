using Jump_Bruteforcer;
using System.Windows;
using Xunit.Abstractions;

namespace TestBrute
{
    /// <summary>
    /// How far off is the default search from the true optimum across the small screens, and does
    /// Optimal mode stay at or below it? Analysis tool, not a regression test:
    /// run with --filter FullyQualifiedName~TestOptimalityProbe.
    /// </summary>
    public class TestOptimalityProbe
    {
        private readonly ITestOutputHelper output;

        public TestOptimalityProbe(ITestOutputHelper output)
        {
            if (Application.Current == null) new Application();
            this.output = output;
        }

        public static (int x, double y, int gx, int gy, string name)[] Cases =
        {
            (410, 407.4, 476, 343, "tomo"),
            (410, 407.4, 518, 503, "sjump"),
            (419, 407.4, 476, 407, "floor"),
            (452, 407.4, 482, 343, "minif"),
            (410, 407.4, 491, 407, "double"),
            (401, 407.4, 380, 343, "double_plane"),
            (401, 407.4, 413, 263, "45"),
            (410, 407.4, 485, 407, "co"),
            (410, 407.4, 443, 311, "leehe"),
            (409, 407.1, 388, 351, "squished"),
            (410, 407.4, 551, 407, "65"),
            (410, 407.4, 485, 407, "groundex15"),
            (389, 407.4, 356, 311, "badl"),
            (420, 407.4, 477, 375, "ground_dplane"),
            (410, 407.4, 452, 279, "32px"),
            (410, 407.4, 450, 311, "the_stupid"),
            (388, 407.4, 541, 407, "platform_invert"),
            (399, 487.4, 399, 295, "platform_teleport"),
            (399, 487.4, 399, 295, "platform_elevator"),
            (401, 407.4, 476, 343, "tomo_2"),
            (401, 407.4, 687, 211, "gate"),
            (17, 343, 179, 471, "ex_rz"),
            (401, 407.4, 587, 407, "needle_extremity_2_7"),
            (401, 407.4, 602, 407, "dt"),
        };

        [Fact]
        [Trait("Category", "Bench")]
        public void ProbeOptimality()
        {
            output.WriteLine("map,astarFrames,optimalFrames,gap,astarVisited,astarMs,optimalMs");
            foreach (var c in Cases)
            {
                Map map = Parser.Parse(File.ReadAllText(@$"..\..\..\jmaps\{c.name}.jmap"));
                Run(c, map, "astar", new Search((c.x, c.y), (c.gx, c.gy), map.CollisionMap));
                Run(c, map, "optimal", new Search((c.x, c.y), (c.gx, c.gy), map.CollisionMap) { UseLayeredBfs = true });
            }
        }

        private void Run((int x, double y, int gx, int gy, string name) c, Map map, string config, Search s)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            SearchResult r = s.RunAStar();
            sw.Stop();
            if (config == "astar")
            {
                _astarFrames = r.Success ? Frames(r) : -1;
                _astarVisited = s.NodesVisited;
                _astarMs = sw.ElapsedMilliseconds;
                return;
            }

            int optimal = r.Success ? Frames(r) : -1;
            output.WriteLine($"{c.name},{_astarFrames},{optimal},{_astarFrames - optimal},{_astarVisited},{_astarMs},{sw.ElapsedMilliseconds}");
        }

        private int _astarFrames;
        private string _astarVisited = "";
        private long _astarMs;

        private static int Frames(SearchResult result) =>
            int.Parse(System.Text.RegularExpressions.Regex.Match(result.InputString, @"Frames: (\d+)").Groups[1].Value);
    }
}
