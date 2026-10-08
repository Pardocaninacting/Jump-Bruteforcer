namespace Jump_Bruteforcer
{
    /// <summary>
    /// Admissible lower bound on the number of frames left, shared by the node based and the
    /// layered search.
    ///
    /// The relaxation allows three pixels horizontally and ten vertically per frame and only
    /// requires the landing pixel to be free, which is a superset of what the real movement rules
    /// allow, so the resulting distances can never overestimate. Note that the historical table in
    /// <see cref="Search"/> is not admissible: it allows ten pixels upwards (the real maximum is
    /// 8.1) but only eight downwards (the real maximum is 9.4), so it can overestimate in
    /// fall heavy sections and the A* is then no longer guaranteed to return the shortest path.
    /// </summary>
    public static class AdmissibleDistance
    {
        public const int VerticalReach = 10;

        public static uint[] Build(CollisionMap map, (int x, int y) goal)
        {
            uint[] table = new uint[Map.WIDTH * Map.HEIGHT];
            Array.Fill(table, uint.MaxValue);
            bool[] seen = new bool[Map.WIDTH * Map.HEIGHT];
            var current = new List<int>();
            var next = new List<int>();

            void Seed(int x, int y)
            {
                if ((uint)x >= Map.WIDTH || (uint)y >= Map.HEIGHT) return;
                int index = y * Map.WIDTH + x;
                if (seen[index]) return;
                seen[index] = true;
                table[index] = 0;
                current.Add(index);
            }

            Seed(goal.x, goal.y);
            Seed(goal.x - 1, goal.y);
            Seed(goal.x + 1, goal.y);
            foreach ((int x, int y) in map.goalPixels) Seed(x, y);

            uint distance = 1;
            while (current.Count > 0)
            {
                next.Clear();
                foreach (int index in current)
                {
                    int px = index % Map.WIDTH, py = index / Map.WIDTH;
                    int minX = Math.Max(px - PhysicsParams.WALKING_SPEED, 0);
                    int maxX = Math.Min(px + PhysicsParams.WALKING_SPEED, Map.WIDTH - 1);
                    int minY = Math.Max(py - VerticalReach, 0);
                    int maxY = Math.Min(py + VerticalReach, Map.HEIGHT - 1);
                    for (int x = minX; x <= maxX; x++)
                    {
                        for (int y = minY; y <= maxY; y++)
                        {
                            int neighbour = y * Map.WIDTH + x;
                            if (seen[neighbour]) continue;
                            if ((map.Collision[x, y] & (CollisionType.Solid | CollisionType.Killer)) != CollisionType.None) continue;
                            seen[neighbour] = true;
                            table[neighbour] = distance;
                            next.Add(neighbour);
                        }
                    }
                }
                (current, next) = (next, current);
                distance++;
            }

            return table;
        }

        /// <summary>
        /// Lower bound for a fractional position: the smaller of the two neighbouring rows, plus the
        /// horizontal component (at most three pixels per frame, and the goal tolerates one pixel).
        /// </summary>
        public static uint At(uint[] table, int x, double y, (int x, int y) goal)
        {
            if ((uint)x >= Map.WIDTH) return uint.MaxValue;
            int lo = Math.Clamp((int)Math.Floor(y), 0, Map.HEIGHT - 1);
            int hi = Math.Clamp((int)Math.Ceiling(y), 0, Map.HEIGHT - 1);
            uint vertical = Math.Min(table[lo * Map.WIDTH + x], table[hi * Map.WIDTH + x]);
            if (vertical == uint.MaxValue) return uint.MaxValue;
            int dx = Math.Abs(x - goal.x);
            uint horizontal = (uint)(Math.Max(0, dx - 1) / PhysicsParams.WALKING_SPEED);
            return Math.Max(vertical, horizontal);
        }
    }
}
