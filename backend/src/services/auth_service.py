"""
Authentication Service
Purpose: signup, signin, signout, validate_session, refresh_token methods using Neon Postgres
Date: 2025-12-13
"""

import logging
import uuid
from datetime import datetime, timedelta
from typing import Optional

from ..core.config import settings
from ..core.security import (
    create_access_token,
    hash_password,
    hash_token,
    verify_password,
)
from ..models.auth import AuditLogCreate, AuthSessionCreate, SigninRequest, SignupRequest
from ..models.user import (
    UserCreate,
    UserHardwareBackgroundCreate,
    UserProfileCreate,
    UserSoftwareBackgroundCreate,
)
from ..services.neon import db

logger = logging.getLogger(__name__)


class AuthService:
    """Authentication service for user management."""

    async def signup(
        self, signup_data: SignupRequest, ip_address: Optional[str] = None, user_agent: Optional[str] = None
    ) -> tuple[dict, str]:
        """
        Create new user account with profile and background information.

        Args:
            signup_data: Signup request data
            ip_address: Client IP address for audit logging
            user_agent: Client user agent for audit logging

        Returns:
            Tuple of (user_data, jwt_token)

        Raises:
            ValueError: If email already exists or validation fails
        """
        # Check if email already exists
        existing_user = await db.execute_one(
            "SELECT id FROM users WHERE email = %s", (signup_data.email,)
        )
        if existing_user:
            await self._log_audit_event(
                event_type="signup_failed",
                user_id=None,
                details={"email": signup_data.email, "reason": "Email already exists"},
                ip_address=ip_address,
                user_agent=user_agent,
            )
            raise ValueError("An account with this email already exists. Please sign in instead.")

        # Validate experience level
        valid_levels = ["Beginner", "Intermediate", "Advanced", "Expert"]
        if signup_data.experience_level not in valid_levels:
            raise ValueError(f"Experience level must be one of: {', '.join(valid_levels)}")

        # Hash password
        hashed_pwd = hash_password(signup_data.password)

        # Create user
        user_id = str(uuid.uuid4())
        await db.execute_write(
            """
            INSERT INTO users (id, email, hashed_password, full_name)
            VALUES (%s, %s, %s, %s)
            """,
            (user_id, signup_data.email, hashed_pwd, signup_data.full_name),
        )

        # Create user profile
        await db.execute_write(
            """
            INSERT INTO user_profiles (user_id, experience_level)
            VALUES (%s, %s)
            """,
            (user_id, signup_data.experience_level),
        )

        # Create software background
        import json
        await db.execute_write(
            """
            INSERT INTO user_software_background (user_id, programming_languages, frameworks)
            VALUES (%s, %s, %s)
            """,
            (
                user_id,
                json.dumps(signup_data.programming_languages),
                json.dumps(signup_data.frameworks),
            ),
        )

        # Create hardware background
        await db.execute_write(
            """
            INSERT INTO user_hardware_background (user_id, available_hardware, robotics_hardware)
            VALUES (%s, %s, %s)
            """,
            (
                user_id,
                json.dumps(signup_data.available_hardware),
                json.dumps(signup_data.robotics_hardware),
            ),
        )

        # Create JWT token
        access_token = create_access_token(data={"sub": user_id})

        # Create session
        await self._create_session(user_id, access_token)

        # Log success
        await self._log_audit_event(
            event_type="signup_success",
            user_id=user_id,
            details={"email": signup_data.email},
            ip_address=ip_address,
            user_agent=user_agent,
        )

        logger.info(f"User signup successful: {signup_data.email}")

        # Return user data
        user_data = {
            "id": user_id,
            "email": signup_data.email,
            "full_name": signup_data.full_name,
        }

        return user_data, access_token

    async def signin(
        self, signin_data: SigninRequest, ip_address: Optional[str] = None, user_agent: Optional[str] = None
    ) -> tuple[dict, str]:
        """
        Authenticate user with email and password.

        Args:
            signin_data: Signin request data
            ip_address: Client IP address for audit logging
            user_agent: Client user agent for audit logging

        Returns:
            Tuple of (user_data, jwt_token)

        Raises:
            ValueError: If credentials are invalid
        """
        logger.info(f"[DEBUG] Signin attempt for: {signin_data.email}")

        # Get user by email
        user = await db.execute_one(
            "SELECT id, email, hashed_password, full_name FROM users WHERE email = %s",
            (signin_data.email,),
        )

        logger.info(f"[DEBUG] User query result: {bool(user)}")

        if not user:
            logger.info(f"[DEBUG] User not found: {signin_data.email}")
            await self._log_audit_event(
                event_type="signin_failed",
                user_id=None,
                details={"email": signin_data.email, "reason": "User not found"},
                ip_address=ip_address,
                user_agent=user_agent,
            )
            raise ValueError("Invalid email or password")

        user_id, email, hashed_password, full_name = user
        # Convert UUID to string for JSON serialization
        user_id = str(user_id)
        logger.info(f"[DEBUG] User found: {user_id}, verifying password...")

        # Verify password
        password_valid = verify_password(signin_data.password, hashed_password)
        logger.info(f"[DEBUG] Password verification result: {password_valid}")

        if not password_valid:
            logger.info(f"[DEBUG] Password verification failed for: {signin_data.email}")
            await self._log_audit_event(
                event_type="signin_failed",
                user_id=user_id,
                details={"email": signin_data.email, "reason": "Invalid password"},
                ip_address=ip_address,
                user_agent=user_agent,
            )
            raise ValueError("Invalid email or password")

        # Create JWT token with extended expiration if remember_me
        expires_delta = None
        if signin_data.remember_me:
            expires_delta = timedelta(days=30)  # 30 days for remember me

        access_token = create_access_token(data={"sub": user_id}, expires_delta=expires_delta)

        # Create session
        await self._create_session(user_id, access_token, signin_data.remember_me)

        # Log success
        await self._log_audit_event(
            event_type="signin_success",
            user_id=user_id,
            details={"email": signin_data.email, "remember_me": signin_data.remember_me},
            ip_address=ip_address,
            user_agent=user_agent,
        )

        logger.info(f"User signin successful: {signin_data.email}")

        # Return user data
        user_data = {
            "id": user_id,
            "email": email,
            "full_name": full_name,
        }

        return user_data, access_token

    async def signout(self, user_id: str, token: str):
        """
        Sign out user by deleting session.

        Args:
            user_id: User ID
            token: Session token to delete
        """
        token_hash_value = hash_token(token)

        # Delete session
        await db.execute_write(
            "DELETE FROM auth_sessions WHERE user_id = %s AND token_hash = %s",
            (user_id, token_hash_value),
        )

        # Log signout
        await self._log_audit_event(
            event_type="signout",
            user_id=user_id,
            details={},
            ip_address=None,
            user_agent=None,
        )

        logger.info(f"User signout successful: {user_id}")

    async def get_current_user(self, user_id: str) -> Optional[dict]:
        """
        Get current user data with profile.

        Args:
            user_id: User ID

        Returns:
            User data dict or None
        """
        # Get user
        user = await db.execute_one(
            "SELECT id, email, full_name, created_at FROM users WHERE id = %s", (user_id,)
        )

        if not user:
            return None

        user_id, email, full_name, created_at = user
        # Convert UUID to string for JSON serialization
        user_id = str(user_id)

        # Get profile
        profile = await db.execute_one(
            "SELECT experience_level FROM user_profiles WHERE user_id = %s", (user_id,)
        )

        # Get software background
        software = await db.execute_one(
            "SELECT programming_languages, frameworks FROM user_software_background WHERE user_id = %s",
            (user_id,),
        )

        # Get hardware background
        hardware = await db.execute_one(
            "SELECT available_hardware, robotics_hardware FROM user_hardware_background WHERE user_id = %s",
            (user_id,),
        )

        # Note: Neon Postgres with psycopg3 returns JSON/JSONB columns as already-parsed Python objects
        user_data = {
            "id": user_id,
            "email": email,
            "full_name": full_name,
            "created_at": created_at.isoformat(),
            "experience_level": profile[0] if profile else None,
            "programming_languages": software[0] if software else [],  # Already a list
            "frameworks": software[1] if software else [],  # Already a list
            "available_hardware": hardware[0] if hardware else [],  # Already a list
            "robotics_hardware": hardware[1] if hardware else [],  # Already a list
        }

        return user_data

    async def refresh_token(self, user_id: str, old_token: str) -> str:
        """
        Refresh JWT token by creating new session.

        Args:
            user_id: User ID
            old_token: Old session token

        Returns:
            New JWT token
        """
        # Delete old session
        old_token_hash = hash_token(old_token)
        await db.execute_write(
            "DELETE FROM auth_sessions WHERE user_id = %s AND token_hash = %s",
            (user_id, old_token_hash),
        )

        # Create new token
        new_token = create_access_token(data={"sub": user_id})

        # Create new session
        await self._create_session(user_id, new_token)

        logger.info(f"Token refreshed for user: {user_id}")

        return new_token

    async def _create_session(self, user_id: str, token: str, remember_me: bool = False):
        """Create auth session in database."""
        session_id = str(uuid.uuid4())
        token_hash_value = hash_token(token)

        # Calculate expiration
        if remember_me:
            expires_at = datetime.utcnow() + timedelta(days=30)
        else:
            expires_at = datetime.utcnow() + timedelta(hours=settings.session_expiration_hours)

        await db.execute_write(
            """
            INSERT INTO auth_sessions (session_id, user_id, token_hash, expires_at)
            VALUES (%s, %s, %s, %s)
            """,
            (session_id, user_id, token_hash_value, expires_at),
        )

    async def _log_audit_event(
        self,
        event_type: str,
        user_id: Optional[str],
        details: dict,
        ip_address: Optional[str],
        user_agent: Optional[str],
    ):
        """Log authentication event to audit_logs table."""
        import json

        audit_id = str(uuid.uuid4())
        await db.execute_write(
            """
            INSERT INTO audit_logs (id, user_id, event_type, details, ip_address, user_agent)
            VALUES (%s, %s, %s, %s, %s, %s)
            """,
            (audit_id, user_id, event_type, json.dumps(details), ip_address, user_agent),
        )


# Global auth service instance
auth_service = AuthService()
