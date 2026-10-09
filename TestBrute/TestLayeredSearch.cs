using FluentAssertions;
using Jump_Bruteforcer;
using System.Windows;
using Xunit.Abstractions;

namespace TestBrute
{
    /// <summary>
    /// The layered grouped BFS must never return a worse path than the node based A*, and on the
    /// screens where both are exhaustive it must return exactly the same frame count.
    /// </summary>
    public class TestLayeredSearch
    {
        private readonly ITestOutputHelper output;

        public TestLayeredSearch(ITestOutputHelper output)
        {
            if (Application.Current == null)
                new Application();
            this.output = output;
        }

        [Theory]
        [InlineData(410, 407.4, 485, 407, "co")]
        [InlineData(410, 407.4, 476, 343, "tomo")]
        [InlineData(452, 407.4, 482, 343, "minif")]
        [InlineData(420, 407.4, 477, 375, "ground_dplane")]
        [InlineData(410, 407.4, 491, 407, "double")]
        [InlineData(410, 407.4, 551, 407, "65")]
        [InlineData(388, 407.4, 541, 407, "platform_invert")]
        // solved by touching a warp, which is nowhere near the goal position
        [InlineData(401, 407.4, 687, 211, "gate")]
        [InlineData(401, 407.4, 587, 407, "needle_extremity_2_7")]
        public void TestLayeredMatchesAStar(int startX, double startY, int goalX, int goalY, string mapName)
        {
            Map map = Parser.Parse(File.ReadAllText(@$"..\..\..\jmaps\{mapName}.jmap"));

            Search astar = new((startX, startY), (goalX, goalY), map.CollisionMap);
            SearchResult reference = astar.RunAStar();

            Search layered = new((startX, startY), (goalX, goalY), map.CollisionMap) { UseLayeredBfs = true };
            SearchResult result = layered.RunAStar();

            result.Success.Should().Be(reference.Success);
            string referenceFrames = astar.Strat.Split('\n')[0];
            string layeredFrames = layered.Strat.Split('\n')[0];
            // The layered sweep is exhaustive, so it may find a shorter path than the A*, whose
            // heuristic is not consistent; it must never find a longer one.
            int referenceCount = int.Parse(referenceFrames.Replace("Frames: ", ""));
            int layeredCount = int.Parse(layeredFrames.Replace("Frames: ", ""));
            layeredCount.Should().BeLessThanOrEqualTo(referenceCount, "the layered search must never be worse");
            output.WriteLine($"{mapName}: {referenceFrames} (astar v={astar.NodesVisited} {astar.TimeTaken}) | " +
                             $"lgb v={layered.NodesVisited} {layered.TimeTaken} U={layered.LayeredUpperBound} beam={layered.LayeredBeamSucceeded}");
        }

        [Theory]
        [InlineData(410, 407.4, 476, 343, "tomo")]
        [InlineData(452, 407.4, 482, 343, "minif")]
        [InlineData(420, 407.4, 477, 375, "ground_dplane")]
        public void TestPreferConciseCostsNoFrames(int startX, double startY, int goalX, int goalY, string mapName)
        {
            Map map = Parser.Parse(File.ReadAllText(@$"..\..\..\jmaps\{mapName}.jmap"));

            Search firstFound = new((startX, startY), (goalX, goalY), map.CollisionMap) { UseLayeredBfs = true, PreferConcise = false };
            SearchResult before = firstFound.RunAStar();

            Search concise = new((startX, startY), (goalX, goalY), map.CollisionMap) { UseLayeredBfs = true };
            SearchResult after = concise.RunAStar();

            after.Success.Should().Be(before.Success);
            // same length, and no longer to write down
            FramesOf(after).Should().Be(FramesOf(before));
            after.Macro.Length.Should().BeLessThanOrEqualTo(before.Macro.Length);
        }

        /// <summary>Frame count reported by a search result.</summary>
        private static int FramesOf(SearchResult result) =>
            int.Parse(System.Text.RegularExpressions.Regex.Match(result.InputString, @"Frames: (\d+)").Groups[1].Value);

        [Fact]
        public void TestLayeredDisableCactus()
        {
            Map map = Parser.Parse(File.ReadAllText(@$"..\..\..\jmaps\co.jmap"));
            Search layered = new((410, 407.4), (485, 407), map.CollisionMap) { UseLayeredBfs = true, DisableCactus = true };
            SearchResult result = layered.RunAStar();
            result.Success.Should().BeTrue();
            layered.Strat.Should().NotContain("+");
        }
    }
}
