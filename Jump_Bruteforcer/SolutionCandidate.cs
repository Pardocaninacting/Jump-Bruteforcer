using System.Windows.Media;

namespace Jump_Bruteforcer
{
    /// <summary>
    /// One of the equally short solutions of a screen. Solutions are only collected when the user
    /// asks for several, and duplicates are dropped by the positions they visit, so the list holds
    /// genuinely different routes rather than the same route written differently.
    /// </summary>
    public sealed class SolutionCandidate
    {
        public SolutionCandidate(int index, int frames, int runs, string macro, string strat, PointCollection points, string fingerprint)
        {
            Index = index;
            Frames = frames;
            Runs = runs;
            Macro = macro;
            Strat = strat;
            Points = points;
            Fingerprint = fingerprint;
        }

        /// <summary>Position in the list, shown to the user.</summary>
        public int Index { get; }
        public int Frames { get; }
        /// <summary>Input changes, i.e. how many runs the notation is written in.</summary>
        public int Runs { get; }
        public string Macro { get; }
        public string Strat { get; }
        public PointCollection Points { get; }
        /// <summary>The positions visited, used to tell two routes apart.</summary>
        public string Fingerprint { get; }

        /// <summary>How the candidate is listed in the UI.</summary>
        public string Label => $"#{Index}  {Frames}f  {Runs} changes";
    }
}
