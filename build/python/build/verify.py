import hashlib
from typing import cast, Any, BinaryIO

def feed_file(h: Any, f: BinaryIO) -> None:
    """Feed data read from an open file into the hashlib instance."""

    while True:
        data = f.read(65536)
        if len(data) == 0:
            # end of file
            break
        h.update(data)

def feed_file_path(h: Any, path: str) -> None:
    """Feed data read from a file (to be opened by this function) into the hashlib instance."""

    with open(path, 'rb') as f:
        feed_file(h, f)

def file_digest(algorithm: Any, path: str) -> str:
    """Calculate the digest of a file and return it in hexadecimal notation."""

    h = algorithm()
    feed_file_path(h, path)
    return cast(str, h.hexdigest())

def guess_digest_algorithm(digest: str) -> Any:
    l = len(digest)
    if l == 32:
        return hashlib.md5
    elif l == 40:
        return hashlib.sha1
    elif l == 64:
        return hashlib.sha256
    elif l == 128:
        return hashlib.sha512
    else:
        return None

def verify_file_digest(path: str, expected_digest: str) -> bool:
    """Verify the digest of a file, and return True if the digest matches with the given expected digest."""
    
    expected_digest = expected_digest.lower()
    algorithm = guess_digest_algorithm(expected_digest)
    assert(algorithm is not None)
    return file_digest(algorithm, path) == expected_digest
