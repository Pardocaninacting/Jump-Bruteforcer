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

        /// <summary>
        /// Vertical table over every pixel plus the horizontal table over every column. Both are
        /// seeded from every pixel the search accepts as a goal, the goal column and the warps, so
        /// neither of them can overestimate the frames left to whichever goal is reached first.
        /// </summary>
        public readonly record struct Table(uint[] Vertical, uint[] Horizontal);

        public static Table Build(CollisionMap map, (int x, int y) goal)
        {
            uint[] table = new uint[Map.WIDTH * Map.HEIGHT];
            Array.Fill(table, uint.MaxValue);
            bool[] seen = new bool[Map.WIDTH * Map.HEIGHT];
            var current = new List<int>();
            var next = new List<int>();
            // columns the search accepts as a goal: the goal tolerates one pixel either side, and
            // touching a warp finishes the screen wherever it is
            bool[] goalColumn = new bool[Map.WIDTH];

            void Seed(int x, int y)
            {
                if ((uint)x >= Map.WIDTH || (uint)y >= Map.HEIGHT) return;
                int index = y * Map.WIDTH + x;
                if (seen[index]) return;
                seen[index] = true;
                table[index] = 0;
                current.Add(index);
            }

            void SeedColumn(int x)
            {
                if ((uint)x < Map.WIDTH) goalColumn[x] = true;
            }

            Seed(goal.x, goal.y);
            Seed(goal.x - 1, goal.y);
            Seed(goal.x + 1, goal.y);
            SeedColumn(goal.x - 1);
            SeedColumn(goal.x);
            SeedColumn(goal.x + 1);
            foreach ((int x, int y) in map.goalPixels)
            {
                Seed(x, y);
                SeedColumn(x);
            }

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

            return new Table(table, BuildHorizontal(goalColumn));
        }

        /// <summary>
        /// Frames needed to cover the horizontal distance alone, at three pixels per frame, from the
        /// nearest goal column. Seeding every warp column is what keeps warp routes from being
        /// pruned by a bound that only knew about the goal position.
        /// </summary>
        private static uint[] BuildHorizontal(bool[] goalColumn)
        {
            uint[] horizontal = new uint[Map.WIDTH];
            Array.Fill(horizontal, uint.MaxValue);
            var queue = new Queue<int>();
            for (int x = 0; x < Map.WIDTH; x++)
            {
                if (goalColumn[x])
                {
                    horizontal[x] = 0;
                    queue.Enqueue(x);
                }
            }

            while (queue.Count > 0)
            {
                int x = queue.Dequeue();
                uint next = horizontal[x] + 1;
                for (int step = 1; step <= PhysicsParams.WALKING_SPEED; step++)
                {
                    int left = x - step;
                    if ((uint)left < Map.WIDTH && horizontal[left] > next)
                    {
                        horizontal[left] = next;
                        queue.Enqueue(left);
                    }
                    int right = x + step;
                    if ((uint)right < Map.WIDTH && horizontal[right] > next)
                    {
                        horizontal[right] = next;
                        queue.Enqueue(right);
                    }
                }
            }
            return horizontal;
        }

        /// <summary>
        /// Lower bound for a fractional position: the smaller of the two neighbouring rows, and the
        /// horizontal distance to the nearest goal column.
        /// </summary>
        public static uint At(Table table, int x, double y)
        {
            if ((uint)x >= Map.WIDTH) return uint.MaxValue;
            int lo = Math.Clamp((int)Math.Floor(y), 0, Map.HEIGHT - 1);
            int hi = Math.Clamp((int)Math.Ceiling(y), 0, Map.HEIGHT - 1);
            uint vertical = Math.Min(table.Vertical[lo * Map.WIDTH + x], table.Vertical[hi * Map.WIDTH + x]);
            if (vertical == uint.MaxValue) return uint.MaxValue;
            uint horizontal = table.Horizontal[x];
            if (horizontal == uint.MaxValue) return uint.MaxValue;
            return Math.Max(vertical, horizontal);
        }
    }
}
