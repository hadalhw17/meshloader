#pragma once
#include <mutex>
#include <compare>
#include <cstdint>
#include <fmt/format.h>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace loader
{

template <typename T>
struct Vec3
{
  Vec3( ) = default;
  Vec3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
  explicit Vec3(T _x) : x(_x), y(_x), z(_x) {}
  explicit Vec3(const T *in)
  {
    x = in[0];
    y = in[1];
    z = in[2];
  }

  auto operator<=>(const Vec3<T> &rhs) const = default;
  template<typename E>
  [[nodiscard]] auto operator+(const Vec3<E> &rhs) const
  {
    return Vec3{ x + static_cast<T>(rhs.x), y + static_cast<T>(rhs.y), z + static_cast<T>(rhs.z) };
  }
  [[nodiscard]] auto operator+(const T rhs) const
  {
    return Vec3{ x + static_cast<T>(rhs), y + static_cast<T>(rhs), z + static_cast<T>(rhs) };
  }
  template<typename E>
  [[nodiscard]] auto operator-(const Vec3<E> &rhs) const
  {
    return Vec3{ x - static_cast<T>(rhs.x), y - static_cast<T>(rhs.y), z - static_cast<T>(rhs.z) };
  }
  template<typename E>
  [[nodiscard]] auto operator*(const Vec3<E> &rhs) const
  {
    return Vec3{ x * static_cast<T>(rhs.x), y * static_cast<T>(rhs.y), z * static_cast<T>(rhs.z) };
  }
  [[nodiscard]] auto operator*(const T rhs) const
  {
    return Vec3{ x * rhs, y * rhs, z * rhs };
  }
  template<typename E>
  [[nodiscard]] auto operator/(const Vec3<E> &rhs) const
  {
    return Vec3{ x / static_cast<T>(rhs.x), y / static_cast<T>(rhs.y), z / static_cast<T>(rhs.z) };
  }
  [[nodiscard]] auto operator/(const T rhs) const
  {
    return Vec3{ x / rhs, y / rhs, z / rhs };
  }
  [[nodiscard]] static Vec3 max(const Vec3 &lhs, const Vec3 &rhs)
  {
    return { std::max(lhs.x, rhs.x), std::max(lhs.y, rhs.y),
             std::max(lhs.z, rhs.z) };
  }

  [[nodiscard]] static Vec3 min(const Vec3<T> &lhs, const Vec3<T> &rhs)
  {
    return { std::min(lhs.x, rhs.x), std::min(lhs.y, rhs.y),
             std::min(lhs.z, rhs.z) };
  }
  template<typename E>
  auto &operator+=(const Vec3<E> &rhs)
  {
    this->x += rhs.x;
    this->y += rhs.y;
    this->z += rhs.z;
    return *this;
  }
  template<typename E>
  friend std::ostream &operator<<(std::ostream &, const Vec3<E> &);

  T x, y, z;
};

using float3 = Vec3<float>;
using uint3 = Vec3<uint32_t>;

struct AABB
{
  float3 min{ 1e+6F, 1e+6F, 1e+6F };
  float3 max{ -1e+6F, -1e+6F, -1e+6F };

  [[nodiscard]] auto extent( ) const { return max - min; }
  auto extend(const float3 &v)
  {
    min = float3::min(min, v);
    max = float3::max(max, v);
  }
};

struct float2
{
  float x, y;

  float2( ) = default;
  float2(float _x, float _y) : x(_x), y(_y) {}
  float2(const float *in)
  {
    x = in[0];
    y = in[1];
  }
  auto operator<=>(const float2 &) const = default;
  friend std::ostream &operator<<(std::ostream &, const float2 &);
};

struct float4
{
  float4( ) = default;
  explicit float4(float3 a) : x(a.x), y(a.y), z(a.z), w(1.f) {}
  float4(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {}
  explicit float4(const float *in)
  {
    x = in[0];
    y = in[1];
    z = in[2];
    w = in[3];
  }
  float x, y, z, w;
};

struct Material
{
  int albedoTexture;
  float4 baseContribution;
  int normalTexture;
  float normalScale;
  int metallicRoughnessTexture;
  float roughness;
  float metallic;

  int occlusionTexture;
  int emissiveTexture;
  float3 emitFactor;
};

struct Primitive
{
  std::uint32_t firstIndex;
  std::uint32_t indexCount;

  int material;
};

struct Mesh
{
  std::vector<float3> positions;
  std::vector<float3> normals;
  std::vector<float2> texCords;

  // 3 components for tangent and 1 for the sign to compute bitangent
  std::vector<float4> tangents;
  std::vector<float3> bitangents;
  std::vector<std::uint32_t> indices;

  std::vector<Primitive> primitives;

  float3 position{ 0.f, 0.f, 0.f };
  // Rotation is defined as euler angles
  float3 rotation{ 0.f, 0.f, 0.f };
  float3 scale{ 1.f, 1.f, 1.f };

  AABB boundingBox;
};

// 4 component image representation
struct Image
{
  std::uint32_t width, height;
  std::vector<unsigned char> image;
  const char *name;
};

struct Model
{
  std::vector<Image> images;
  std::vector<Material> materials;
  std::vector<Mesh> meshes;
};

enum class EDistanceType
{
  FACE = 0, // Hit in the face.
  EDGE1 = 1,// Hit on the first edge.
  EDGE2 = 2,// Hit on the second edge.
  EDGE3 = 3,// Hit on the third edge.
  VERT1 = 4,// Hit on the first vertex.
  VERT2 = 5,// Hit on the second vertex.
  VERT3 = 6 // Hit on the third vertex.
};

struct STriangleDistanceResult
{
  float distance;
  EDistanceType hit_type;
  float3 hit_point;
};

struct Vertex
{
  float3 position;
  float3 normal;
  float2 texCord;

  auto operator<=>(const Vertex &rhs) const = default;
  friend std::ostream &operator<<(std::ostream &, const Vertex &);
};
inline std::ostream &operator<<(std::ostream &os, const float2 &fl)
{
  const auto res = fmt::format("{{ x: {}, y: {}}}", fl.x, fl.y);
  return os << res << std::endl;
}
template<typename T>
inline std::ostream &operator<<(std::ostream &os, const Vec3<T> &fl)
{
  const auto res = fmt::format("{{ x: {}, y: {}, z: {}}}", fl.x, fl.y, fl.z);
  return os << res << std::endl;
}

inline std::ostream &operator<<(std::ostream &os, const Vertex &vert)
{
  return os << "position " << vert.position << " normal " << vert.normal
            << " texture coordinate " << vert.texCord << std::endl;
}

struct VertexHash
{
  std::size_t operator( )(const Vertex &vertex) const
  {
    return static_cast<std::uint32_t>(vertex.position.x * 19965295109.F) ^
           static_cast<std::uint32_t>(vertex.position.y * 18511065037.F) ^
           static_cast<std::uint32_t>(vertex.position.z * 45183875657.F) ^
           static_cast<std::uint32_t>(vertex.normal.x * 34699057009.F) ^
           static_cast<std::uint32_t>(vertex.normal.y * 56587034149.F) ^
           static_cast<std::uint32_t>(vertex.normal.z * 79652433737.F) ^
           static_cast<std::uint32_t>(vertex.texCord.x * 13739426149.F) ^
           static_cast<std::uint32_t>(vertex.texCord.y * 59901554101.F);
  }
};

using Edge = std::pair<Vertex, Vertex>;
using EdgeIndexed = std::pair<std::uint32_t, std::uint32_t>;

struct IndexedEdgeHash
{
  std::size_t operator( )(const EdgeIndexed &edge) const
  {
    auto pack = [](const uint32_t &v1, const uint32_t &v2) {
      uint64_t r = v1;
      r <<= 32;
      r |= v2;
      return r;
    };
    std::uint64_t packed = pack(edge.first, edge.second);
    std::uint64_t v = packed * 3935559000370003845 + 2691343689449507681;

    v ^= v >> 21;
    v ^= v << 37;
    v ^= v >> 4;

    v *= 4768777513237032717;

    v ^= v << 20;
    v ^= v >> 41;
    v ^= v << 5;
    return v;
  }
};

struct EdgeHash
{
  std::size_t operator( )(const Edge &edge) const
  {
    VertexHash hash;
    return hash(edge.first) ^ hash(edge.second);
  }
};

/*
* Contains information about the current job.
* startx - x id of starting batch
* starty - y id of starting batch
* maxx - how many batches to process accross x
* maxy - how many batches to process accross y
*/
struct BatchJob
{
  int startX = -1, startY = -1, startZ = -1;
  int maxX = -1 , maxY = -1, maxZ = -1;
};


/*
* Ring buffer implementation inspired by the one on the lab.
* The whole process is pretty simple, split y-axis ono 2 equal rectangular areas,
* which could either be rendered concurrently, or divided further.
* Pretty much like a hierarchical grid, but hierarchical stripes instead.
*/
class RingBuffer
{
  private:
  int ringSize;
  int head, tail;
  BatchJob* ringBuffer;
  int processedJobs;
  std::recursive_mutex ringLock;
  private:
  int NextPosition(int pos);
  bool IsRingFull();
  bool IsRingEmpty();
  public:
  RingBuffer(int size);
  ~RingBuffer(void);
  BatchJob RemoveFromRing();
  bool AddToRing(BatchJob val);
  bool IsRingEmptyAndJobsCompleted();  //Return whether ring is empty and no jobs are being processed
  void NotifyOfJobCompletion(); //Reduces the count on the number of jobs being processed
};


/*
* Just increament a positino counter, or wrap it to the beginning.
*/
inline int RingBuffer::NextPosition(int pos)
{
  return (pos + 1) % ringSize;
}

/*
* Ring is full if the head is one position behind tail.
*/
inline bool RingBuffer::IsRingFull()
{
  return tail == NextPosition(head);
}

/*
* Ring is empty if head is at the same position as tail.
*/
inline bool RingBuffer::IsRingEmpty()
{
  return head == tail;
}

inline RingBuffer::RingBuffer(int size)
{
  ringSize = size;
  ringBuffer = new BatchJob[size];
  processedJobs = 0;
  head = tail = 0;
}

inline RingBuffer::~RingBuffer(void)
{
  delete[] ringBuffer;
}

/*
* Remove empy job if ring is empty.
*/
inline BatchJob RingBuffer::RemoveFromRing()
{
  ringLock.lock();
  if (IsRingEmpty())
  {
    ringLock.unlock();
    return BatchJob();
  }
  BatchJob val = ringBuffer[tail];
  tail = NextPosition(tail);
  processedJobs++;
  ringLock.unlock();
  return val;
}


/*
* Add job to the ring, in case the rign in not full and the job is actually valid, i.e. has some batches to render
*/
inline bool RingBuffer::AddToRing(BatchJob val)
{
  ringLock.lock();
  if (IsRingFull() || val.startY == val.maxY)
  {
    ringLock.unlock();
    return false;
  }
  ringBuffer[head] = val;
  head = NextPosition(head);
  ringLock.unlock();
  return true;
}


/*
* Should we terminate our main loop?
*/
inline bool RingBuffer::IsRingEmptyAndJobsCompleted()
{
  ringLock.lock();
  bool res = IsRingEmpty() && (processedJobs == 0);
  ringLock.unlock();
  return res;
}


/*
* Decrement the counter of current jobs available.
*/
inline void RingBuffer::NotifyOfJobCompletion()
{
  ringLock.lock();
  processedJobs--;
  ringLock.unlock();
}


}// namespace loader
