#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

// Creates a path if it does not exist. Optionally clears previous content.
inline void makePath(const std::string& path, bool clean = false) {
    fs::path p(path);

    // If the path includes a filename, extract the parent path
    if (p.has_filename()) {
        p = p.parent_path();
    }
    
    // Create the directory (and intermediate directories) if needed
    fs::create_directories(p);
    std::cout << "Directory prepared: " << p << '\n';

    if (clean) {
        // Clear contents of the directory
        for (const auto& entry : fs::directory_iterator(p)) {
            fs::remove_all(entry.path());
            std::cout << "Removed: " << entry.path() << '\n';
        }
    }
}

// Joins two strings to form a valid path.
inline std::string joinPath(const std::string& base, const std::string& sub) {
    return (fs::path(base) / fs::path(sub)).string();
}