typedef struct
{
  uint8_t x;
  uint8_t y;
}Array2D;

typedef struct
{
  uint8_t colorTag;
  int colorCount;
  std::vector<Array2D> bBox(2);
}Cluster;
