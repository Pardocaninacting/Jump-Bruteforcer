using System.Windows.Media;

namespace Jump_Bruteforcer
{
    /// <summary>
    /// One essentially different solution of a screen.
    ///
    /// Candidates are grouped by the vertical structure of their input sequence, because that is
    /// what tells two solutions apart: the horizontal part differs mostly by one frame micro
    /// movements that visit slightly different pixels on the way to the same place, while a
    /// different jump pattern is a different route. One entry per group is kept, written in the
    /// fewest input changes, and how many spellings were folded into it is reported through
    /// <see cref="Variants"/>.
    /// </summary>
    public sealed class SolutionCandidate
    {
        public SolutionCandidate(int index, int frames, int runs, int frameDelta, string macro, string strat,
            PointCollection points, string fingerprint, string structure)
        {
            Index = index;
            Frames = frames;
            Runs = runs;
            FrameDelta = frameDelta;
            Macro = macro;
            Strat = strat;
            Points = points;
            Fingerprint = fingerprint;
            Structure = structure;
        }

        /// <summary>Position in the list, shown to the user.</summary>
        public int Index { get; }
        public int Frames { get; }
        /// <summary>Input changes, i.e. how many runs the notation is written in.</summary>
        public int Runs { get; }
        /// <summary>Frames this solution costs compared to the shortest one collected.</summary>
        public int FrameDelta { get; }
        public string Macro { get; }
        public string Strat { get; }
        public PointCollection Points { get; }
        /// <summary>The positions visited, used to tell two spellings of one route apart.</summary>
        public string Fingerprint { get; }
        /// <summary>The vertical structure, which is what makes two solutions different routes.</summary>
        public string Structure { get; }
        /// <summary>How many spellings of this route were collected.</summary>
        public int Variants { get; set; } = 1;

        /// <summary>How the candidate is listed in the UI. Kept short: the structure can be long.</summary>
        public string Label => FrameDelta > 0
            ? $"#{Index}  {Frames}f (+{FrameDelta}f)  {Runs} changes" + (Variants > 1 ? $"  ({Variants} spellings)" : "")
            : $"#{Index}  {Frames}f  {Runs} changes" + (Variants > 1 ? $"  ({Variants} spellings)" : "");
    }
}
