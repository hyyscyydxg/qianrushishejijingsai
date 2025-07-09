import os
import tos

def upload_to_tos(
    local_path: str,
    *,
    bucket: str,
    object_prefix: str,
    access_key: str,
    secret_key: str,
    endpoint: str = "tos-cn-shanghai.volces.com",
    region: str = "cn-shanghai"
) -> str:
    """
    使用火山引擎 volcengine-tos SDK 上传文件到指定 Bucket。

    :param local_path: 本地文件路径
    :param bucket: TOS 桶名
    :param object_prefix: 对象存储路径+文件名，如 clips/xxx.mp4
    :param access_key: 火山 AK
    :param secret_key: 火山 SK
    :param endpoint: TOS 访问域名
    :param region: 所属区域（如 cn-shanghai
    :return: 可公网访问的对象 URL
    """
    try:
        client = tos.TosClientV2(access_key, secret_key, endpoint, region)
        client.put_object_from_file(bucket, object_prefix, local_path)
        return f"https://{bucket}.{endpoint}/{object_prefix}"

    except tos.exceptions.TosClientError as e:
        print('❌ TOS Client Error:', e.message)
        raise RuntimeError(f"客户端错误: {e.message}") from e

    except tos.exceptions.TosServerError as e:
        print(f'❌ TOS Server Error: {e.code}, RequestId: {e.request_id}')
        raise RuntimeError(f"服务端错误 {e.code}: {e.message}") from e

    except Exception as e:
        print(f"❌ 未知上传错误: {e}")
        raise
